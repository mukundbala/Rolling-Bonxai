#include "bonxai_map/occupancy_map.hpp"

namespace Bonxai {
namespace Occupancy {

// ============================================================================
// OccupancyOptions Implementation
// ============================================================================

OccupancyOptions::OccupancyOptions()
    : prob_miss_log(logods(0.4f)),
      prob_hit_log(logods(0.7f)),
      clamp_min_log(logods(0.12f)),
      clamp_max_log(logods(0.97f)),
      occupancy_threshold_log(logods(0.5f))
{}

// ============================================================================
// Utility Functions
// ============================================================================

int32_t logods(float prob) {
    return static_cast<int32_t>(1e6 * std::log(prob / (1.0f - prob)));
}

float prob(int32_t logods_fixed) {
    float logods = static_cast<float>(logods_fixed) * 1e-6f;
    return 1.0f - 1.0f / (1.0f + std::exp(logods));
}

// ============================================================================
// CellOcc Implementation
// ============================================================================

CellOcc::CellOcc()
    : update_id(4),
      probability_log(UnknownProbability)
{}

} // namespace Occupancy

// ============================================================================
// Constructors & Destructor
// ============================================================================

OccupancyMap::OccupancyMap(double resolution)
    : grid_(resolution),
      options_()
{
    // Reserve memory upfront to prevent vector from resizing + reallocating
    hit_coords_.reserve(1024);
    miss_coords_.reserve(4096);
}

OccupancyMap::OccupancyMap(double resolution, const Occupancy::OccupancyOptions& options)
    : grid_(resolution),
      options_(options)
{
    // Reserve memory upfront to prevent vector from resizing + reallocating
    hit_coords_.reserve(1024);
    miss_coords_.reserve(4096);
}

OccupancyMap::OccupancyMap(const Occupancy::OccupancyOptions& options,
                           VoxelGrid<Occupancy::CellOcc>&& grid)
    : grid_(std::move(grid)),
      options_(options)
{
    hit_coords_.reserve(1024);
    miss_coords_.reserve(4096);
}

// ============================================================================
// Move Semantics
// ============================================================================

OccupancyMap::OccupancyMap(OccupancyMap&& other) noexcept
    : grid_(std::move(other.grid_)),
      options_(other.options_),
      update_count_(other.update_count_),
      hit_coords_(std::move(other.hit_coords_)),
      miss_coords_(std::move(other.miss_coords_)),
      // Don't move accessors - they're invalidated by grid move
      accessor_(std::nullopt),
      const_accessor_(std::nullopt),
      total_insertions_(other.total_insertions_),
      hit_points_added_(other.hit_points_added_),
      miss_points_added_(other.miss_points_added_)
{
    // Reset other's stats
    other.total_insertions_ = 0;
    other.hit_points_added_ = 0;
    other.miss_points_added_ = 0;
    other.update_count_ = 1;
}

OccupancyMap& OccupancyMap::operator=(OccupancyMap&& other) noexcept {
    if (this != &other) {
        grid_ = std::move(other.grid_);
        options_ = other.options_;
        update_count_ = other.update_count_;
        hit_coords_ = std::move(other.hit_coords_);
        miss_coords_ = std::move(other.miss_coords_);
        
        // Invalidate accessors (grid has moved)
        accessor_.reset();
        const_accessor_.reset();
        
        // Move stats
        total_insertions_ = other.total_insertions_;
        hit_points_added_ = other.hit_points_added_;
        miss_points_added_ = other.miss_points_added_;
        
        // Reset other
        other.total_insertions_ = 0;
        other.hit_points_added_ = 0;
        other.miss_points_added_ = 0;
        other.update_count_ = 1;
    }
    return *this;
}

// ============================================================================
// Underlying VoxelGrid Access
// ============================================================================

VoxelGrid<Occupancy::CellOcc>& OccupancyMap::getGrid() noexcept {
    return grid_;
}

const VoxelGrid<Occupancy::CellOcc>& OccupancyMap::getGrid() const noexcept {
    return grid_;
}

// ============================================================================
// Options & Configuration
// ============================================================================

const Occupancy::OccupancyOptions& OccupancyMap::getOptions() const noexcept {
    return options_;
}

void OccupancyMap::setOptions(const Occupancy::OccupancyOptions& options) {
    options_ = options;
}

// ============================================================================
// Accessor Management
// ============================================================================

void OccupancyMap::ensureAccessorValid() {
    if (!accessor_.has_value()) {
        accessor_.emplace(grid_.createAccessor());
    }
}

void OccupancyMap::ensureConstAccessorValid() const {
    if (!const_accessor_.has_value()) {
        const_accessor_.emplace(grid_.createConstAccessor());
    }
}

void OccupancyMap::invalidateAccessors() {
    accessor_.reset();
    const_accessor_.reset();
}

OccupancyMap::Accessor& OccupancyMap::getAccessor() {
    ensureAccessorValid();
    return accessor_.value();
}

OccupancyMap::ConstAccessor OccupancyMap::getConstAccessor() const {
    ensureConstAccessorValid();
    return const_accessor_.value();
}

// ============================================================================
// Statistics
// ============================================================================

size_t OccupancyMap::getActiveCellCount() const noexcept {
    return grid_.activeCellsCount();
}

size_t OccupancyMap::getMemoryUsage() const noexcept {
    return grid_.memUsage();
}

Occupancy::OccupancyMapStats OccupancyMap::getQuickStats() const noexcept {
    Occupancy::OccupancyMapStats stats;
    
    stats.grid_memory_bytes = grid_.memUsage();
    stats.buffer_memory_bytes = hit_coords_.capacity() * sizeof(CoordT) +
                                miss_coords_.capacity() * sizeof(CoordT);
    stats.total_memory_bytes = stats.grid_memory_bytes +
                               stats.buffer_memory_bytes;
    
    stats.total_active_cells = grid_.activeCellsCount();
    stats.total_insertions = total_insertions_;
    stats.hit_points_added = hit_points_added_;
    stats.miss_points_added = miss_points_added_;
    stats.current_update_cycle = update_count_;
    stats.has_data = stats.total_active_cells > 0;
    
    return stats;
}

CoordT OccupancyMap::worldToVoxel(const Vector3D& point) const {
    return grid_.posToCoord(point);
}

void OccupancyMap::applyTemporalDecay(double dt_seconds, double decay_time_seconds) {
    if (dt_seconds <= 0.0 || decay_time_seconds <= 0.0) {
        return;
    }

    ensureAccessorValid();

    const double decay_factor = std::exp(-dt_seconds / decay_time_seconds);
    auto visitor = [this, decay_factor](Occupancy::CellOcc& cell, const CoordT&) {
        if (cell.probability_log == Occupancy::UnknownProbability) {
            return;
        }

        const float current_probability = Occupancy::prob(cell.probability_log);
        // Decay evidence toward "unknown" (p = 0.5), not toward free
        // space.  Multiplying the probability itself by decay_factor made a
        // fresh hit fall below the occupied threshold after only a few timer
        // ticks, which made the dynamic layer appear empty.
        const float target_probability = std::max(
            1.0e-6f,
            std::min(
                1.0f - 1.0e-6f,
                0.5f + (current_probability - 0.5f) * static_cast<float>(decay_factor))
        );

        cell.probability_log = std::clamp(
            Occupancy::logods(target_probability),
            options_.clamp_min_log,
            options_.clamp_max_log
        );
    };

    grid_.forEachCell(visitor);
}

Occupancy::OccupancyMapStats OccupancyMap::getStats(bool compute_expensive) const {
    Occupancy::OccupancyMapStats stats = getQuickStats();
    
    if (!compute_expensive || !stats.has_data) {
        return stats;
    }
    
    // Expensive computations requiring full grid iteration
    bool first = true;
    int64_t prob_sum = 0;
    
    auto visitor = [&](const Occupancy::CellOcc& cell, const CoordT& coord) {
        if (first) {
            stats.min_coord = coord;
            stats.max_coord = coord;
            stats.min_probability_log = cell.probability_log;
            stats.max_probability_log = cell.probability_log;
            first = false;
        } else {
            stats.min_coord.x = std::min(stats.min_coord.x, coord.x);
            stats.min_coord.y = std::min(stats.min_coord.y, coord.y);
            stats.min_coord.z = std::min(stats.min_coord.z, coord.z);
            
            stats.max_coord.x = std::max(stats.max_coord.x, coord.x);
            stats.max_coord.y = std::max(stats.max_coord.y, coord.y);
            stats.max_coord.z = std::max(stats.max_coord.z, coord.z);
            
            stats.min_probability_log = std::min(stats.min_probability_log,
                                                 cell.probability_log);
            stats.max_probability_log = std::max(stats.max_probability_log,
                                                 cell.probability_log);
        }
        
        prob_sum += cell.probability_log;
        
        if (cell.probability_log > options_.occupancy_threshold_log) {
            stats.occupied_cells++;
        } else if (cell.probability_log < options_.occupancy_threshold_log) {
            stats.free_cells++;
        } else {
            stats.unknown_cells++;
        }
    };
    
    grid_.forEachCell(visitor);
    
    // Derived metrics
    if (stats.total_active_cells > 0) {
        stats.mean_probability_log = static_cast<int32_t>(
            prob_sum / static_cast<int64_t>(stats.total_active_cells)
        );
        
        size_t known_cells = stats.occupied_cells + stats.free_cells;
        if (known_cells > 0) {
            stats.occupancy_ratio = static_cast<float>(stats.occupied_cells) /
                                   static_cast<float>(known_cells);
        }
        
        stats.exploration_ratio = static_cast<float>(known_cells) /
                                 static_cast<float>(stats.total_active_cells);
    }
    
    return stats;
}

// ============================================================================
// Query Methods (const)
// ============================================================================

bool OccupancyMap::isOccupied(const CoordT& coord) const {
    auto accessor = getConstAccessor();
    if (const auto* cell = accessor.value(coord)) {
        return cell->probability_log > options_.occupancy_threshold_log;
    }
    return false;
}

bool OccupancyMap::isUnknown(const CoordT& coord) const {
    auto accessor = getConstAccessor();
    if (const auto* cell = accessor.value(coord)) {
        return cell->probability_log == options_.occupancy_threshold_log;
    }
    return true;  // Unknown cells are truly unknown
}

bool OccupancyMap::isFree(const CoordT& coord) const {
    auto accessor = getConstAccessor();
    if (const auto* cell = accessor.value(coord)) {
        return cell->probability_log < options_.occupancy_threshold_log;
    }
    return false;
}

void OccupancyMap::getOccupiedVoxels(std::vector<CoordT>& coords) const {
    coords.clear();
    
    auto visitor = [&](const Occupancy::CellOcc& cell, const CoordT& coord) {
        if (cell.probability_log > options_.occupancy_threshold_log) {
            coords.push_back(coord);
        }
    };
    
    grid_.forEachCell(visitor);
}

void OccupancyMap::getFreeVoxels(std::vector<CoordT>& coords) const {
    coords.clear();
    
    auto visitor = [&](const Occupancy::CellOcc& cell, const CoordT& coord) {
        if (cell.probability_log < options_.occupancy_threshold_log) {
            coords.push_back(coord);
        }
    };
    
    grid_.forEachCell(visitor);
}

// ============================================================================
// Update Methods (non-const)
// ============================================================================

void OccupancyMap::addHitPoint(const Vector3D& point) {
    addHitPoint(grid_.posToCoord(point));
}

void OccupancyMap::addHitPoint(const CoordT& coord) {
    ensureAccessorValid();
    ++total_insertions_;
    ++hit_points_added_;
    
    Occupancy::CellOcc* cell = accessor_->value(coord, true);
    
    if (cell->update_id != update_count_) {
        cell->probability_log = std::min(
            cell->probability_log + options_.prob_hit_log,
            options_.clamp_max_log
        );
        cell->update_id = update_count_;
        hit_coords_.push_back(coord);
    }
}

void OccupancyMap::resetPoint(const CoordT& coord) {
    ensureAccessorValid();
    Occupancy::CellOcc* cell = accessor_->value(coord, true);
    // isUnknown() is defined relative to the configured occupancy threshold,
    // which is not guaranteed to be bit-identical to UnknownProbability.
    cell->probability_log = options_.occupancy_threshold_log;
    // Make the cell eligible for an update in the current insertion cycle.
    cell->update_id = 4;
}

void OccupancyMap::addMissPoint(const Vector3D& point) {
    addMissPoint(grid_.posToCoord(point));
}

void OccupancyMap::addMissPoint(const CoordT& coord) {
    ensureAccessorValid();
    ++total_insertions_;
    ++miss_points_added_;
    
    Occupancy::CellOcc* cell = accessor_->value(coord, true);
    
    if (cell->update_id != update_count_) {
        cell->probability_log = std::max(
            cell->probability_log + options_.prob_miss_log,
            options_.clamp_min_log
        );
    }
    
    cell->update_id = update_count_;
    miss_coords_.push_back(coord);
}

void OccupancyMap::updateFreeCells(const Vector3D& origin) {
    ensureAccessorValid();
    
    auto clearPoint = [this](const CoordT& coord) {
        Occupancy::CellOcc* cell = accessor_->value(coord, true);
        if (cell->update_id != update_count_) {
            cell->probability_log = std::max(
                cell->probability_log + options_.prob_miss_log,
                options_.clamp_min_log
            );
            cell->update_id = update_count_;
        }
        return true;
    };
    
    const auto coord_origin = grid_.posToCoord(origin);
    
    for (const auto& coord_end : hit_coords_) {
        Occupancy::RayIterator(coord_origin, coord_end, clearPoint);
    }
    hit_coords_.clear();
    
    for (const auto& coord_end : miss_coords_) {
        Occupancy::RayIterator(coord_origin, coord_end, clearPoint);
    }
    miss_coords_.clear();
    
    incrementUpdateCount();
}

void OccupancyMap::updateFreeCellsPreRayTrace() {
    if (hit_coords_.empty() && miss_coords_.empty()) {
        return;
    }

    // Clear these bad boys
    hit_coords_.clear();
    miss_coords_.clear();

    // Increment update count
    incrementUpdateCount();
}

void OccupancyMap::incrementUpdateCount() noexcept {
    if (++update_count_ == 4) {
        update_count_ = 1;
    }
}

// ============================================================================
// Memory Management
// ============================================================================

void OccupancyMap::releaseUnusedMemory() {
    grid_.releaseUnusedMemory();
    invalidateAccessors();  // Must recreate both accessors
    
    // Shrink oversized buffers
    if (hit_coords_.capacity() > 2048) {
        std::vector<CoordT>().swap(hit_coords_);
        hit_coords_.reserve(1024);
    }
    if (miss_coords_.capacity() > 8192) {
        std::vector<CoordT>().swap(miss_coords_);
        miss_coords_.reserve(4096);
    }
}

void OccupancyMap::shrinkToFit() {
    hit_coords_.shrink_to_fit();
    miss_coords_.shrink_to_fit();
}

// ============================================================================
// Performance Counters
// ============================================================================

void OccupancyMap::resetCounters() noexcept {
    total_insertions_ = 0;
    hit_points_added_ = 0;
    miss_points_added_ = 0;
}

} // namespace Bonxai
