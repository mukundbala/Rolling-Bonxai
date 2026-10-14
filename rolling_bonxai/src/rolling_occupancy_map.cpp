#include "rolling_bonxai/rolling_occupancy_map.hpp"



namespace RollingBonxai
{
RollingOccupancyMap::RollingOccupancyMap(const AllParameters& params, std::shared_ptr<Logger> logger)
    :
    params_(params),
    logger_(logger)
{
    // Build factories from MapParams
    buildFactories();

    // Initialize coordinate system
    c_sys_ = std::make_unique<ChunkCoordinateSystem>(params_.map_params.resolution,params_.chunk_size);

    // Initialise transition manager
    transition_manager_ = std::make_unique<TransitionManager>(params_.hysteresis_ratio, *c_sys_);

    // Initialise the loading policy (both spellings accepted; the param namespace is "neighborhood")
    if (params_.loading_policy_name == "neighbourhood" || params_.loading_policy_name == "neighborhood") {
        loading_policy_ = std::make_unique<NeighborhoodPolicy>(params_.nb_radius);
    }
    else if (params_.loading_policy_name == "planar") {
        loading_policy_ = std::make_unique<PlanarNeighborhoodPolicy>(
            params_.nb_radius,
            params_.planar_z_min,
            params_.planar_z_max,
            params_.planar_use_relative);
    }
    else if (params_.loading_policy_name == "temporal") {
        loading_policy_ = std::make_unique<TemporalNeighborhoodPolicy>(
            params_.nb_radius,
            params_.temporal_age_weightage,
            std::chrono::seconds(params_.temporal_max_age_s));
    }
    else {
        logger_->log_warn("[RollingBonxai]: Unknown loading policy '" + params_.loading_policy_name +
            "' (expected neighbourhood, planar or temporal); falling back to neighbourhood");
        loading_policy_ = std::make_unique<NeighborhoodPolicy>(params_.nb_radius);
    }

    // Initialise the file storage backend
    std::filesystem::path storage_path(params_.full_save_folder_path);
    storage_backend_ = std::make_unique<FileStorageBackend>(storage_path);
    [[maybe_unused]] bool storage_ok = storage_backend_->initStorageBackend();

    // Create the async I/O manager (takes ownership of storage_backend_)
    asyncio_manager_ = std::make_unique<AsyncChunkManager>(
        std::move(storage_backend_), params_.asyncio);
    
    std::ostringstream log_oss;
    log_oss << "[RollingBonxai: Initialised!]\n" <<
    "VoxelResolution: " << std::to_string(params_.map_params.resolution) << "\n" <<  
    "ChunkSize: " << std::to_string(params_.chunk_size) << "\n" <<
    "Hysteresis:" << std::to_string(params_.hysteresis_ratio) << "\n" <<
    "LoadEvictPolicy: " << loading_policy_->getName() << "\n" << 
    "IO Enabled: " << params_.asyncio.enable_io << "\n";

    logger_->log_info(log_oss.str());
}

RollingOccupancyMap::~RollingOccupancyMap()
{
    // AsyncChunkManager destructor handles thread shutdown.
    // Dirty chunks should be flushed by updateRolling / graceful shutdown
    // before reaching this point. Log a warning if there are still active chunks.
    if (!active_chunks_.empty()) {
        logger_->log_warn("[RollingBonxai]: RollingOccupancyMap destroyed with " 
            + std::to_string(active_chunks_.size()) + " active chunks remaining");
    }
}

// ============================================================================
// Factory Construction
// ============================================================================

void RollingOccupancyMap::buildFactories()
{
    // Capture the values we need by copy (safe for lambda lifetime)
    const double resolution = params_.map_params.resolution;
    const float sensor_hit = static_cast<float>(params_.map_params.sensor_hit);
    const float sensor_miss = static_cast<float>(params_.map_params.sensor_miss);
    const float sensor_min = static_cast<float>(params_.map_params.sensor_min);
    const float sensor_max = static_cast<float>(params_.map_params.sensor_max);
    const float occ_threshold = static_cast<float>(params_.map_params.occupancy_threshold);

    // Options factory: builds OccupancyOptions from MapParams sensor model
    options_factory_ = [sensor_hit, sensor_miss, sensor_min, sensor_max, occ_threshold]()
        -> Bonxai::Occupancy::OccupancyOptions 
    {
        Bonxai::Occupancy::OccupancyOptions opts;
        opts.prob_hit_log = Bonxai::Occupancy::logods(sensor_hit);
        opts.prob_miss_log = Bonxai::Occupancy::logods(sensor_miss);
        opts.clamp_min_log = Bonxai::Occupancy::logods(sensor_min);
        opts.clamp_max_log = Bonxai::Occupancy::logods(sensor_max);
        opts.occupancy_threshold_log = Bonxai::Occupancy::logods(occ_threshold);
        return opts;
    };

    // Map factory: constructs OccupancyMap with resolution + options
    // Captures options_factory_ by reference (this outlives all factory calls)
    map_factory_ = [resolution, this]() -> std::unique_ptr<Bonxai::OccupancyMap> 
    {
        auto opts = options_factory_();
        return std::make_unique<Bonxai::OccupancyMap>(resolution, opts);
    };
}

// ============================================================================
// Occupancy Update
// ============================================================================

void RollingOccupancyMap::updateOccupancy(const std::vector<Vector3D>& points,
                                          const Vector3D& source,
                                          double max_range) 
{
    // defensive, this shouldnt be the case.
    if (points.empty()) {
        logger_->log_warn("[RollingBonxai]: Pointcloud is empty in updateOccupancy");
        return;
    }

    // square of max range for quick comparisons
    const double max_range_sq = max_range * max_range;

    // Convert the source to voxel coordinate
    CoordT source_vox_coord = c_sys_->positionToVoxelCoordinate(source);
    
    // Convert the source to chunk coordinate
    ChunkCoord source_chunk_coord = c_sys_->positionToChunkCoordinate(source);
    
    // precompute bounds in terms of voxel coordinates. Each 
    const auto [source_voxel_min,source_voxel_max] = c_sys_->worldBoundsToVoxelBounds(source_chunk_coord);
    auto current_voxel_min = source_voxel_min;
    auto current_voxel_max = source_voxel_max;

    // Current ray chunk. This will keep track as we walk along the ray
    ChunkCoord current_ray_chunk_coord = source_chunk_coord;

    // Touched chunks
    std::unordered_set<ChunkCoord, ChunkCoordHash> touched_chunks;

    for (const auto& point : points) {
        // ----------------------------------------------------------------
        // Step 1: Classify hit vs miss
        // ----------------------------------------------------------------
        
        // Vector from source to the point
        Vector3D source2pointvec = point - source;

        // Compute the distance squared
        double dist_squared_source2point = source2pointvec.squaredNorm();

        Vector3D end_point;
        bool is_hit;

        if (dist_squared_source2point >= max_range_sq) {
            // need to clip it
            end_point = source + (source2pointvec / std::sqrt(dist_squared_source2point)) * max_range;
            is_hit = false;
        }
        else {
            end_point = point;
            is_hit = true;
        }

        // Convert endpoint to a voxel coord
        CoordT endpoint_vox_coord = c_sys_->positionToVoxelCoordinate(end_point);

        // Get the chunk coordinate where the endpoint lies
        ChunkCoord endpoint_chunk_coord = c_sys_->positionToChunkCoordinate(end_point);

        // ----------------------------------------------------------------
        // Step 2: Dispatch the endpoint to the appropriate chunk
        // ----------------------------------------------------------------
        ManagedChunk* endpoint_chunk = active_chunks_.find(endpoint_chunk_coord);

        // Check if the end point chunk is valid
        if (!endpoint_chunk || !endpoint_chunk->isMapValid()) {
            ++dropped_points_;
            continue; //onwards to the next point
        }

        // Grab a pointer to the map from the endpoint chunk
        Bonxai::OccupancyMap* endpoint_mutable_map = endpoint_chunk->getMutableMap();

        if (is_hit) {
            endpoint_mutable_map->addHitPoint(endpoint_vox_coord);
        } else {
            endpoint_mutable_map->addMissPoint(endpoint_vox_coord);
        }
        touched_chunks.insert(endpoint_chunk_coord);

        // ----------------------------------------------------------------
        // Step 3: Raytrace source → endpoint, dispatch intermediates
        // ----------------------------------------------------------------
        // Reset ray-walk state to source chunk for each ray
        current_voxel_min = source_voxel_min;
        current_voxel_max = source_voxel_max;
        current_ray_chunk_coord = source_chunk_coord;

        Bonxai::Occupancy::RayIterator(
            source_vox_coord, endpoint_vox_coord,
            [&](const CoordT& coord) -> bool 
            {
                // Boundary-check optimization: 6 integer comparisons
                if (coord.x < current_voxel_min.x || coord.x >= current_voxel_max.x ||
                    coord.y < current_voxel_min.y || coord.y >= current_voxel_max.y ||
                    coord.z < current_voxel_min.z || coord.z >= current_voxel_max.z)
                {
                    // Crossed a chunk boundary — recompute
                    // Convert voxel coord back to world position for chunk lookup
                    Vector3D current_ray_pos = c_sys_->voxelToPositionCoordinate(coord);
                    current_ray_chunk_coord = c_sys_->positionToChunkCoordinate(current_ray_pos);
                    auto bounds = c_sys_->worldBoundsToVoxelBounds(current_ray_chunk_coord);
                    current_voxel_min = bounds.first;
                    current_voxel_max = bounds.second;
                }

                // Find the chunk and dispatch the miss point
                ManagedChunk* ray_managed = active_chunks_.find(current_ray_chunk_coord);
                if (ray_managed && ray_managed->isMapValid()) {
                    ray_managed->getMutableMap()->addMissPoint(coord);
                    touched_chunks.insert(current_ray_chunk_coord);
                }
                // If chunk not loaded, silently skip this voxel
                // (no dropped_points_ increment — only endpoints count as drops)

                return true; // Continue ray iteration
            }
        );
    }
    // ----------------------------------------------------------------
    // Step 4: Finalise all touched chunks
    // ----------------------------------------------------------------
    for (const auto& chunk_coord : touched_chunks) {
        ManagedChunk* managed = active_chunks_.find(chunk_coord);
        if (managed && managed->isMapValid()) {
            managed->getMutableMap()->updateFreeCellsPreRayTrace();
        }
    }
}

// ============================================================================
// Rolling Update 
// ============================================================================

void RollingOccupancyMap::updateRolling(
    const Vector3D& source,
    const Vector3D& source_lin_vel)
{
    // ------------------------------------------------------------------
    // Step 1: Always poll pending loads — promote completed futures
    // ------------------------------------------------------------------
    pollPendingLoads();

    // ------------------------------------------------------------------
    // Step 2: Evaluate transition manager
    // ------------------------------------------------------------------
    TransitionManagerContext tm_context;
    tm_context.ego_position = source;
    tm_context.ego_velocity = source_lin_vel;

    bool transition_triggered = transition_manager_->shouldTriggerTransition(tm_context);
    
    // ------------------------------------------------------------------
    // Step 3: If transition triggered, perform load/evict cycle
    // ------------------------------------------------------------------
    if (transition_triggered) {
        const ChunkCoord ref_chunk = transition_manager_->getRefChunkCoord();
        logger_->log_info("[RollingBonxai]: Transition required into chunk ("
                    + std::to_string(ref_chunk.x) + "," + std::to_string(ref_chunk.y) + ","
                    + std::to_string(ref_chunk.z) + ")");
        performTransition(ref_chunk, source);
    }

    // ------------------------------------------------------------------
    // Step 4: Update metadata for the current reference chunk
    // ------------------------------------------------------------------
    const ChunkCoord current_chunk = transition_manager_->getRefChunkCoord();
    auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    auto& meta = chunk_metadata_[current_chunk];
    meta.last_access_time = now_ns;
    meta.access_count++;
}

// ============================================================================
// Poll Pending Loads
// ============================================================================

void RollingOccupancyMap::pollPendingLoads()
{
    // We cannot erase from the map while iterating, so collect completed keys
    std::vector<ChunkCoord> completed;

    for (auto& [coord, future] : pending_loads_) {
        // Non-blocking check: is the future ready?
        if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            continue;
        }

        // Future is ready — try to retrieve the result
        try {
            // load it
            std::unique_ptr<Bonxai::OccupancyMap> loaded_map = future.get();

            if (loaded_map) 
            {
                // Don't insert if something already occupies this slot
                // (edge case: rapid transitions could cause this)
                if (!active_chunks_.contains(coord)) 
                {
                    // Wrap in ManagedChunk. Chunks loaded from disk start clean;
                    // chunks created fresh by the factory have no data so also clean.
                    // The occupancy update path will mark them dirty when modified.
                    ManagedChunk managed(std::move(loaded_map), coord, /*dirty=*/false);
                    active_chunks_.insert(coord, std::move(managed));

                    // Initialise metadata if not already present
                    if (chunk_metadata_.find(coord) == chunk_metadata_.end()) {
                        auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count();
                        ChunkMetadata meta;
                        meta.creation_time = now_ns;
                        meta.last_access_time = now_ns;
                        meta.access_count = 1;
                        chunk_metadata_[coord] = meta;
                    }
                    logger_->log_info("[RollingBonxai]: Loading to active -> chunk ("
                        + std::to_string(coord.x) + "," + std::to_string(coord.y) + ","
                        + std::to_string(coord.z) + ")");
                } 
                else 
                {
                    logger_->log_warn("pollPendingLoads: chunk ("
                        + std::to_string(coord.x) + "," + std::to_string(coord.y) + ","
                        + std::to_string(coord.z) + ") already active — discarding loaded map");
                }
            } 
            else {
                logger_->log_warn("pollPendingLoads: null map returned for chunk ("
                    + std::to_string(coord.x) + "," + std::to_string(coord.y) + ","
                    + std::to_string(coord.z) + ")");
            }
        } catch (const std::exception& e) {
            logger_->log_error("pollPendingLoads: load failed for chunk ("
                + std::to_string(coord.x) + "," + std::to_string(coord.y) + ","
                + std::to_string(coord.z) + "): " + e.what());
        }

        completed.push_back(coord);
    }

    // Erase completed entries
    for (const auto& coord : completed) {
        pending_loads_.erase(coord);
    }
}

// ============================================================================
// Perform Transition (Load / Evict Cycle)
// ============================================================================

void RollingOccupancyMap::performTransition(const ChunkCoord& ref_chunk,
                                            const Vector3D& source)
{
    // Get the time now
    const auto now = std::chrono::steady_clock::now();

    // Build the policy context
    PolicyContext policy_ctx(source, ref_chunk, now, chunk_metadata_);

    // ------------------------------------------------------------------
    // A. Determine the desired chunk set
    // ------------------------------------------------------------------
    // The current chunk is ALWAYS loaded with maximum priority
    std::unordered_set<ChunkCoord, ChunkCoordHash> desired_set;
    desired_set.insert(ref_chunk);

    // Iterate all neighbours within the policy radius
    const int radius = params_.nb_radius;
    ChunkCoordinateSystem::forEachNeighbour(ref_chunk, radius,
        [&](const ChunkCoord& candidate) {
            if (loading_policy_->shouldLoad(candidate, policy_ctx)) {
                desired_set.insert(candidate);
            }
        }
    );

    // ------------------------------------------------------------------
    // B. Load: dispatch async loads for desired chunks not yet active/pending
    // ------------------------------------------------------------------

    // Current chunk gets priority 2.0 (always above policy range of [0,1])
    // so it is loaded first
    if (!active_chunks_.contains(ref_chunk) &&
        pending_loads_.find(ref_chunk) == pending_loads_.end() &&
        !asyncio_manager_->isLoading(ref_chunk))
    {
        auto future_opt = asyncio_manager_->loadAsync(
            ref_chunk, /*priority=*/2.0f, map_factory_, options_factory_);
        if (future_opt.has_value()) {
            pending_loads_[ref_chunk] = std::move(future_opt.value());
        }
    }
    // Remaining desired chunks at policy-determined priorities
    for (const auto& coord : desired_set) {
        // Skip the current chunk (already handled above)
        if (coord == ref_chunk) {
            continue;
        }

        // Skip if already active or already loading
        if (active_chunks_.contains(coord)) {
            continue;
        }
        if (pending_loads_.find(coord) != pending_loads_.end()) {
            continue;
        }
        if (asyncio_manager_->isLoading(coord)) {
            continue;
        }

        float priority = static_cast<float>(loading_policy_->getPriority(coord, policy_ctx));

        auto future_opt = asyncio_manager_->loadAsync(
            coord, priority, map_factory_, options_factory_);
        if (future_opt.has_value()) {
            pending_loads_[coord] = std::move(future_opt.value());
        }
    }
    logger_->log_info("[RollingBonxai]: Async Loads Issued!");
    // ------------------------------------------------------------------
    // C. Evict: save-if-dirty, then remove chunks the policy no longer wants
    // ------------------------------------------------------------------
    // Collect eviction candidates first (cannot erase during forEachChunk)
    std::vector<ChunkCoord> to_evict;

    active_chunks_.forEachChunk(
        [&](const ChunkCoord& coord, [[maybe_unused]] const ManagedChunk& chunk) {
            // Never evict the current reference chunk
            if (coord == ref_chunk) {
                return;
            }
            if (loading_policy_->shouldEvict(coord, policy_ctx)) {
                to_evict.push_back(coord);
            }
        }
    );

    for (const auto& coord : to_evict) {
        ManagedChunk* managed = active_chunks_.find(coord);
        if (!managed) {
            continue; // shouldn't happen, defensive
        }

        // If dirty, save the VoxelGrid before evicting
        if (managed->isDirty() && managed->isMapValid()) {
            // Transfer ownership of the OccupancyMap out of the ManagedChunk
            // then move the VoxelGrid out of the OccupancyMap for saving.
            // After transferMapOwnership(), the ManagedChunk's map_ is nullptr.
            auto map_uptr = managed->transferMapOwnership();
            if (map_uptr) {
                // Move the grid out for the save pipeline
                [[maybe_unused]] auto future_bool = 
                asyncio_manager_->saveAsync(coord, std::move(map_uptr->getGrid()));
            }
            logger_->log_info("[RollingBonxai]: (Direct) Saving dirty chunk ("
            + std::to_string(coord.x) + "," + std::to_string(coord.y) + ","
            + std::to_string(coord.z) + ")");
            
        }
        else{
            logger_->log_info("[RollingBonxai]: (Lazy) Not saving clean chunk ("
            + std::to_string(coord.x) + "," + std::to_string(coord.y) + ","
            + std::to_string(coord.z) + ")");
        }

        // Remove from active storage (ManagedChunk destroyed here)
        active_chunks_.erase(coord);

        // Note: we intentionally keep chunk_metadata_ for this coord.
        // The temporal policy may need historical metadata for future decisions.
    }
    logger_->log_info("[RollingBonxai]: Async saves issued!");

    logger_->log_info("[RollingBonxai]: performTransition: ref=("
        + std::to_string(ref_chunk.x) + "," + std::to_string(ref_chunk.y) + ","
        + std::to_string(ref_chunk.z) + ") desired=" + std::to_string(desired_set.size())
        + " active-chunks  =" + std::to_string(active_chunks_.size())
        + " pending-loads  =" + std::to_string(pending_loads_.size())
        + " evicted-chunks =" + std::to_string(to_evict.size()));
}

// ============================================================================
// Statistics (aggregated across all active chunks)
// ============================================================================

bool RollingOccupancyMap::isStatsStable() const noexcept {
    //Only safe when there are no inflight loads and pending saves
    return pending_loads_.empty() && asyncio_manager_->pendingSaveCount() == 0;
}

size_t RollingOccupancyMap::getActiveCellCount() const noexcept
{
    size_t total = 0;
    active_chunks_.forEachChunk(
        [&total]([[maybe_unused]] const ChunkCoord&, const ManagedChunk& chunk) {
            if (const auto* map = chunk.getConstMap()) {
                total += map->getActiveCellCount();
            }
        }
    );
    return total;
}

size_t RollingOccupancyMap::getActiveChunkCount() const {
    size_t total = 0;
    active_chunks_.forEachChunk(
        [&total]([[maybe_unused]] const ChunkCoord&, const ManagedChunk& chunk) {
            if (chunk.getConstMap()) {
                total += 1;
            }
        }
    );
    return total;
}

size_t RollingOccupancyMap::getDirtyChunkCount() const {
    size_t total = 0;
    active_chunks_.forEachChunk(
        [&total]([[maybe_unused]] const ChunkCoord&, const ManagedChunk& chunk) {
            if (chunk.getConstMap() && chunk.isDirty()) {
                total += 1;
            }
        }
    );
    return total;
}

size_t RollingOccupancyMap::getCleanChunkCount() const {
    size_t total = 0;
    active_chunks_.forEachChunk(
        [&total]([[maybe_unused]] const ChunkCoord&, const ManagedChunk& chunk) {
            if (chunk.getConstMap() && !chunk.isDirty()) {
                total += 1;
            }
        }
    );
    return total;
}

std::string RollingOccupancyMap::getTransitionState() const {
    auto t_state_enum = transition_manager_->getRefTransitionState();
    std::string reflected = reflectTransitionState(t_state_enum);
    return reflected;
}

std::vector<std::pair<RollingOccupancyMap::Vector3D,bool>> RollingOccupancyMap::getChunkStates() const {
    std::vector<std::pair<Vector3D,bool>> ret;
    ret.reserve(active_chunks_.size());
    
    active_chunks_.forEachChunk(
        [this,&ret](const ChunkCoord &coord, const ManagedChunk& chunk) {
            if (chunk.getConstMap()) {
                auto pos = this->c_sys_->chunkToPositionCoordinate(coord);
                bool dirty = chunk.isDirty();
                ret.emplace_back(pos,dirty);
            }
        }
    );
    return ret;
}

std::vector<RollingOccupancyMap::Vector3D> RollingOccupancyMap::getPendingChunks() const {
    std::vector<Vector3D> pending_chunk_coords;
    pending_chunk_coords.reserve(pending_loads_.size());

    for (const auto&[chunk_coord,future] : pending_loads_) {
        auto pos = this->c_sys_->chunkToPositionCoordinate(chunk_coord);
        pending_chunk_coords.push_back(pos);
    }

    return pending_chunk_coords;
}

size_t RollingOccupancyMap::getMemoryUsage() const noexcept
{
    size_t total = 0;
    active_chunks_.forEachChunk(
        [&total]([[maybe_unused]] const ChunkCoord&, const ManagedChunk& chunk) {
            if (const auto* map = chunk.getConstMap()) {
                total += map->getMemoryUsage();
            }
        }
    );
    return total;
}

Bonxai::Occupancy::OccupancyMapStats RollingOccupancyMap::getQuickStats() const noexcept
{
    Bonxai::Occupancy::OccupancyMapStats agg;

    active_chunks_.forEachChunk(
        [&agg](const ChunkCoord&, const ManagedChunk& chunk) {
            if (const auto* map = chunk.getConstMap()) {
                auto cs = map->getQuickStats();

                agg.grid_memory_bytes += cs.grid_memory_bytes;
                agg.buffer_memory_bytes += cs.buffer_memory_bytes;
                agg.total_memory_bytes += cs.total_memory_bytes;
                agg.total_active_cells += cs.total_active_cells;
                agg.total_insertions += cs.total_insertions;
                agg.hit_points_added += cs.hit_points_added;
                agg.miss_points_added += cs.miss_points_added;

                if (cs.has_data) {
                    agg.has_data = true;
                }
            }
        }
    );

    return agg;
}

Bonxai::Occupancy::OccupancyMapStats RollingOccupancyMap::getStats(bool compute_expensive) const
{
    if (!compute_expensive) {
        return getQuickStats();
    }

    Bonxai::Occupancy::OccupancyMapStats agg;
    bool first_chunk_with_data = true;

    // Accumulators for global mean calculation
    int64_t global_prob_sum = 0;
    size_t global_known_cells = 0;

    active_chunks_.forEachChunk(
        [&](const ChunkCoord&, const ManagedChunk& chunk) {
            if (const auto* map = chunk.getConstMap()) {
                auto cs = map->getStats(true);

                // Sum memory
                agg.grid_memory_bytes += cs.grid_memory_bytes;
                agg.buffer_memory_bytes += cs.buffer_memory_bytes;
                agg.total_memory_bytes += cs.total_memory_bytes;

                // Sum cell counts
                agg.total_active_cells += cs.total_active_cells;
                agg.occupied_cells += cs.occupied_cells;
                agg.free_cells += cs.free_cells;
                agg.unknown_cells += cs.unknown_cells;

                // Sum counters
                agg.total_insertions += cs.total_insertions;
                agg.hit_points_added += cs.hit_points_added;
                agg.miss_points_added += cs.miss_points_added;

                if (cs.has_data) {
                    agg.has_data = true;

                    // Global bounding box (min of mins, max of maxes)
                    if (first_chunk_with_data) {
                        agg.min_coord = cs.min_coord;
                        agg.max_coord = cs.max_coord;
                        agg.min_probability_log = cs.min_probability_log;
                        agg.max_probability_log = cs.max_probability_log;
                        first_chunk_with_data = false;
                    } else {
                        agg.min_coord.x = std::min(agg.min_coord.x, cs.min_coord.x);
                        agg.min_coord.y = std::min(agg.min_coord.y, cs.min_coord.y);
                        agg.min_coord.z = std::min(agg.min_coord.z, cs.min_coord.z);

                        agg.max_coord.x = std::max(agg.max_coord.x, cs.max_coord.x);
                        agg.max_coord.y = std::max(agg.max_coord.y, cs.max_coord.y);
                        agg.max_coord.z = std::max(agg.max_coord.z, cs.max_coord.z);

                        agg.min_probability_log = std::min(agg.min_probability_log, cs.min_probability_log);
                        agg.max_probability_log = std::max(agg.max_probability_log, cs.max_probability_log);
                    }

                    // Accumulate for global mean
                    global_prob_sum += static_cast<int64_t>(cs.mean_probability_log) 
                                       * static_cast<int64_t>(cs.total_active_cells);
                    global_known_cells += cs.occupied_cells + cs.free_cells;
                }
            }
        }
    );

    // Compute derived global metrics
    if (agg.total_active_cells > 0) {
        agg.mean_probability_log = static_cast<int32_t>(
            global_prob_sum / static_cast<int64_t>(agg.total_active_cells));

        if (global_known_cells > 0) {
            agg.occupancy_ratio = static_cast<float>(agg.occupied_cells) /
                                  static_cast<float>(global_known_cells);
        }

        agg.exploration_ratio = static_cast<float>(global_known_cells) /
                                static_cast<float>(agg.total_active_cells);
    }

    return agg;
}

// ============================================================================
// Memory Management
// ============================================================================

void RollingOccupancyMap::releaseUnusedMemory()
{
    // Note: We access the map via getMutableMap() which marks chunks dirty.
    // This is a known trade-off — releaseUnusedMemory modifies the grid structure
    // (frees leaf nodes) so it genuinely is a mutation, but it doesn't change
    // occupancy data. For correctness of save semantics, this is acceptable:
    // a chunk that had memory released will be re-saved with the compacted grid.
    active_chunks_.forEachChunk(
        [](const ChunkCoord&, ManagedChunk& chunk) {
            if (chunk.isMapValid()) {
                chunk.getMutableMap()->releaseUnusedMemory();
            }
        }
    );
    logger_->log_info("[RollingBonxai]: Releasing Unused Memory!");
}

void RollingOccupancyMap::shrinkToFit()
{
    // shrinkToFit only shrinks internal vectors (hit_coords_, miss_coords_),
    // which are transient buffers. Same dirty-marking caveat applies.
    active_chunks_.forEachChunk(
        [](const ChunkCoord&, ManagedChunk& chunk) {
            if (chunk.isMapValid()) {
                chunk.getMutableMap()->shrinkToFit();
            }
        }
    );
    logger_->log_info("[RollingBonxai]: Shrinking buffers!");
}

// ============================================================================
// Query Interface
// ============================================================================

bool RollingOccupancyMap::isOccupied(const Vector3D& position) const
{
    const ChunkCoord chunk = c_sys_->positionToChunkCoordinate(position);
    const ManagedChunk* managed = active_chunks_.find(chunk);
    if (!managed || !managed->getConstMap()) {
        return false;
    }

    const CoordT coord = managed->getConstMap()->getGrid().posToCoord(
        position.x(), position.y(), position.z());
    return managed->getConstMap()->isOccupied(coord);
}

bool RollingOccupancyMap::isOccupied(const CoordT& coord) const
{
    const Vector3D pos= c_sys_->voxelToPositionCoordinate(coord);
    const ChunkCoord chunk = c_sys_->positionToChunkCoordinate(pos);
    const ManagedChunk* managed = active_chunks_.find(chunk);
    if (!managed || !managed->getConstMap()) {
        return false;
    }
    return managed->getConstMap()->isOccupied(coord);
}

bool RollingOccupancyMap::isFree(const Vector3D& position) const
{
    const ChunkCoord chunk = c_sys_->positionToChunkCoordinate(position);
    const ManagedChunk* managed = active_chunks_.find(chunk);
    if (!managed || !managed->getConstMap()) {
        return false;
    }

    const CoordT coord = managed->getConstMap()->getGrid().posToCoord(
        position.x(), position.y(), position.z());
    return managed->getConstMap()->isFree(coord);
}

bool RollingOccupancyMap::isFree(const CoordT& coord) const
{
    const Vector3D pos= c_sys_->voxelToPositionCoordinate(coord);
    const ChunkCoord chunk = c_sys_->positionToChunkCoordinate(pos);
    const ManagedChunk* managed = active_chunks_.find(chunk);
    if (!managed || !managed->getConstMap()) {
        return false;
    }
    return managed->getConstMap()->isFree(coord);
}

bool RollingOccupancyMap::isUnknown(const Vector3D& position) const
{
    const ChunkCoord chunk = c_sys_->positionToChunkCoordinate(position);
    const ManagedChunk* managed = active_chunks_.find(chunk);
    if (!managed || !managed->getConstMap()) {
        return true; // Unloaded chunk → unknown
    }

    const CoordT coord = managed->getConstMap()->getGrid().posToCoord(
        position.x(), position.y(), position.z());
    return managed->getConstMap()->isUnknown(coord);
}

bool RollingOccupancyMap::isUnknown(const CoordT& coord) const
{
    
    const Vector3D pos= c_sys_->voxelToPositionCoordinate(coord);
    const ChunkCoord chunk = c_sys_->positionToChunkCoordinate(pos);
    const ManagedChunk* managed = active_chunks_.find(chunk);
    if (!managed || !managed->getConstMap()) {
        return true; // Unloaded chunk → unknown
    }
    return managed->getConstMap()->isUnknown(coord);
}

void RollingOccupancyMap::getOccupiedVoxels(std::vector<CoordT>& coords) const
{
    coords.clear();

    std::vector<CoordT> chunk_coords;
    active_chunks_.forEachChunk(
        [&coords, &chunk_coords](const ChunkCoord&, const ManagedChunk& chunk) {
            if (const auto* map = chunk.getConstMap()) {
                chunk_coords.clear();
                map->getOccupiedVoxels(chunk_coords);
                coords.insert(coords.end(), chunk_coords.begin(), chunk_coords.end());
            }
        }
    );
}

void RollingOccupancyMap::getFreeVoxels(std::vector<CoordT>& coords) const
{
    coords.clear();

    std::vector<CoordT> chunk_coords;
    active_chunks_.forEachChunk(
        [&coords, &chunk_coords](const ChunkCoord&, const ManagedChunk& chunk) {
            if (const auto* map = chunk.getConstMap()) {
                chunk_coords.clear();
                map->getFreeVoxels(chunk_coords);
                coords.insert(coords.end(), chunk_coords.begin(), chunk_coords.end());
            }
        }
    );
}

// ============================================================================
// Accessors
// ============================================================================

const ChunkCoordinateSystem& RollingOccupancyMap::getCoordinateSystem() const
{
    return *c_sys_;
}

const HashMapChunkStorage& RollingOccupancyMap::getActiveChunks() const
{
    return active_chunks_;
}

ChunkCoord RollingOccupancyMap::getCurrentChunk() const
{
    return transition_manager_->getRefChunkCoord();
}

uint64_t RollingOccupancyMap::getDroppedPointCount() const noexcept
{
    return dropped_points_;
}

const RollingOccupancyMap::AllParameters& RollingOccupancyMap::getParams() const noexcept
{
    return params_;
}

const RollingOccupancyMap::OccupancyMapFactory& RollingOccupancyMap::getMapFactory() const noexcept
{
    return map_factory_;
}

const RollingOccupancyMap::OccupancyOptionsFactory& RollingOccupancyMap::getOptionsFactory() const noexcept
{
    return options_factory_;
}

} //namespace RollingBonxai