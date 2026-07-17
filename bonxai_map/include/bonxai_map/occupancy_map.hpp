#pragma once

#include <cstdint>
#include <vector>
#include <optional>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Geometry>

#include "bonxai_core/bonxai.hpp"

namespace Bonxai {
namespace Occupancy {

/**
 * @brief Occupancy probability options using log-odds representation
 * 
 * Log-odds representation allows efficient Bayesian updates:
 * log_odds(p) = log(p / (1-p))
 * 
 * Updates: log_odds_new = log_odds_old + log_odds_measurement
 */
struct OccupancyOptions {
    int32_t prob_miss_log;              ///< Log-odds for ray miss (default: logods(0.4))
    int32_t prob_hit_log;               ///< Log-odds for ray hit (default: logods(0.7))
    int32_t clamp_min_log;              ///< Min probability clamp (default: logods(0.12))
    int32_t clamp_max_log;              ///< Max probability clamp (default: logods(0.97))
    int32_t occupancy_threshold_log;    ///< Occupied/free threshold (default: logods(0.5))
    
    OccupancyOptions();
};

/**
 * @brief Convert probability to log-odds (fixed-point, 1e6 scale)
 * @param prob Probability in range (0, 1), exclusive
 * @return Log-odds as int32_t (scaled by 1e6 for fixed-point arithmetic)
 * @note Handles boundary cases: prob near 0 or 1 will saturate
 */
[[nodiscard]] int32_t logods(float prob);

/**
 * @brief Convert log-odds to probability
 * @param logods_fixed Log-odds in fixed-point representation (1e6 scale)
 * @return Probability in range [0, 1]
 */
[[nodiscard]] float prob(int32_t logods_fixed);

/**
 * @brief Voxel cell data for occupancy mapping
 * 
 * Uses update_id to track which cells have been updated in current cycle,
 * preventing multiple updates to the same cell in one batch.
 */
struct CellOcc {
    int32_t update_id{4};                   ///< Update cycle ID (cycles: 1→2→3→1...)
    int32_t probability_log{28};            ///< Occupancy log-odds (default: unknown)
    
    CellOcc();
};

/// Unknown probability constant (logods(0.5) ≈ 0, stored as 28 after scaling artifacts)
constexpr int32_t UnknownProbability = 28;

/**
 * @brief Comprehensive statistics about occupancy map state
 */
struct OccupancyMapStats {
    // Memory usage
    size_t total_memory_bytes{0};
    size_t grid_memory_bytes{0};
    size_t buffer_memory_bytes{0};
    
    // Cell counts
    size_t total_active_cells{0};
    size_t occupied_cells{0};
    size_t free_cells{0};
    size_t unknown_cells{0};
    
    // Update statistics
    uint64_t total_insertions{0};
    uint64_t hit_points_added{0};
    uint64_t miss_points_added{0};
    uint8_t current_update_cycle{0};
    
    // Spatial extent (bounding box)
    CoordT min_coord{0, 0, 0};
    CoordT max_coord{0, 0, 0};
    bool has_data{false};
    
    // Probability distribution
    int32_t min_probability_log{0};
    int32_t max_probability_log{0};
    int32_t mean_probability_log{0};
    
    // Derived metrics
    float occupancy_ratio{0.0f};        ///< occupied / (occupied + free)
    float exploration_ratio{0.0f};      ///< (occupied + free) / total_active
};

/**
 * @brief Iterate along a ray between two voxel coordinates using DDA algorithm
 * 
 * @tparam Functor Function object with signature: bool(const CoordT&)
 * @param key_origin Ray start coordinate (inclusive)
 * @param key_end Ray end coordinate (exclusive)
 * @param func Functor called for each voxel along ray. Return false to stop iteration.
 * 
 * @note Uses 3D Digital Differential Analyzer (DDA) for efficient ray traversal
 * @complexity O(manhattan_distance(origin, end))
 */
template <class Functor>
inline void RayIterator(const CoordT& key_origin, const CoordT& key_end, const Functor& func) {
    if (key_origin == key_end) { return; }
    if (!func(key_origin)) { return; }
    
    CoordT error{0, 0, 0};
    CoordT coord = key_origin;
    CoordT delta = key_end - coord;
    const CoordT step{
        delta.x < 0 ? -1 : 1,
        delta.y < 0 ? -1 : 1,
        delta.z < 0 ? -1 : 1
    };
    
    delta = {
        delta.x < 0 ? -delta.x : delta.x,
        delta.y < 0 ? -delta.y : delta.y,
        delta.z < 0 ? -delta.z : delta.z
    };
    
    const int max = std::max({delta.x, delta.y, delta.z});
    
    for (int i = 0; i < max - 1; ++i) {
        error = error + delta;
        
        if ((error.x << 1) >= max) {
            coord.x += step.x;
            error.x -= max;
        }
        
        if ((error.y << 1) >= max) {
            coord.y += step.y;
            error.y -= max;
        }
        
        if ((error.z << 1) >= max) {
            coord.z += step.z;
            error.z -= max;
        }
        
        if (!func(coord)) { return; }
    }
}

/**
 * @brief Compute all voxel coordinates along a ray
 * 
 * @param key_origin Ray start coordinate
 * @param key_end Ray end coordinate
 * @param ray Output vector of coordinates (cleared before filling)
 */
inline void ComputeRay(const CoordT& key_origin, const CoordT& key_end, std::vector<CoordT>& ray) {
    ray.clear();
    
    RayIterator(key_origin, key_end, [&ray](const CoordT& coord) {
        ray.push_back(coord);
        return true;
    });
}

} // namespace Occupancy

/**
 * @class OccupancyMap
 * @brief Probabilistic 3D occupancy grid using Bonxai sparse voxel representation
 * 
 * This class manages a 3D occupancy map where each voxel stores occupancy probability
 * using log-odds representation. The underlying storage is sparse (Bonxai VoxelGrid),
 * only allocating memory for observed voxels.
 * 
 * Key features:
 * - Sparse storage (only observed voxels consume memory)
 * - Efficient ray tracing for sensor models
 * - Log-odds representation for numerically stable Bayesian updates
 * - Move-only semantics (copying disabled due to large memory footprint)
 * - Automatic accessor management with invalidation safety
 * - Separate mutable/const accessors for proper const correctness
 * 
 * @performance
 * - Insertion: ~50-100 µs per point (including ray trace)
 * - Query: ~50-100 ns per voxel (with accessor caching)
 * - Memory: ~20-40 bytes per active voxel
 * 
 * @thread_safety Not thread-safe. External synchronization required for concurrent access.
 * 
 * @note After calling releaseUnusedMemory(), internal accessors are invalidated
 *       and will be automatically recreated on next use.
 * 
 * @example
 * @code
 * // Create map
 * OccupancyMap map(0.1);  // 10cm resolution
 * 
 * // Insert point cloud
 * std::vector<Eigen::Vector3d> points = lidar.getPoints();
 * Eigen::Vector3d sensor_origin = robot.getPosition();
 * map.insertPointCloud(points, sensor_origin, 30.0);  // 30m max range
 * 
 * // Query occupancy
 * if (map.isOccupied(map.getGrid().posToCoord(query_point))) {
 *     // Obstacle detected
 * }
 * @endcode
 */
class OccupancyMap {
public:
    using Vector3D = Eigen::Vector3d;
    using Accessor = VoxelGrid<Occupancy::CellOcc>::Accessor;
    using ConstAccessor = VoxelGrid<Occupancy::CellOcc>::ConstAccessor;

    // ========================================================================
    // Construction & Destruction
    // ========================================================================
    
    /**
     * @brief Construct occupancy map with default options
     * @param resolution Voxel size in meters
     */
    explicit OccupancyMap(double resolution);
    
    /**
     * @brief Construct occupancy map with custom options
     * @param resolution Voxel size in meters
     * @param options Occupancy probability settings
     */
    explicit OccupancyMap(double resolution, const Occupancy::OccupancyOptions& options);
    
    /**
     * @brief Construct from existing grid (for deserialization)
     * @param options Occupancy probability settings
     * @param grid Existing VoxelGrid (moved)
     */
    explicit OccupancyMap(const Occupancy::OccupancyOptions& options, 
                         VoxelGrid<Occupancy::CellOcc>&& grid);
    
    ~OccupancyMap() = default;
    
    // No copying (move-only)
    OccupancyMap(const OccupancyMap&) = delete;
    OccupancyMap& operator=(const OccupancyMap&) = delete;
    
    // Move semantics
    OccupancyMap(OccupancyMap&& other) noexcept;
    OccupancyMap& operator=(OccupancyMap&& other) noexcept;
    
    // ========================================================================
    // Grid Access
    // ========================================================================
    
    /**
     * @brief Get mutable reference to underlying grid
     * @return VoxelGrid reference
     * @warning Modifying grid structure may invalidate internal accessors
     */
    [[nodiscard]] VoxelGrid<Occupancy::CellOcc>& getGrid() noexcept;
    
    /**
     * @brief Get const reference to underlying grid
     * @return Const VoxelGrid reference
     */
    [[nodiscard]] const VoxelGrid<Occupancy::CellOcc>& getGrid() const noexcept;
    
    // ========================================================================
    // Options & Configuration
    // ========================================================================
    
    /**
     * @brief Get current occupancy options
     * @return Const reference to options
     */
    [[nodiscard]] const Occupancy::OccupancyOptions& getOptions() const noexcept;
    
    /**
     * @brief Set occupancy options
     * @param options New options to use
     * @note Does not affect existing voxel probabilities
     */
    void setOptions(const Occupancy::OccupancyOptions& options);

    // ========================================================================
    // Coordinate System Conversion from 3D Point Coordinate to Grid Coordinate
    // ========================================================================
    
    template <typename PositionCoordinateT>
    CoordT positionToVoxelCoordinate(const PositionCoordinateT &pos_coord)
    {
        // Convert our point to something we know
        const auto position_3d_eigen = ConvertPoint<Vector3D>(pos_coord);
        
        CoordT voxel_coord = grid_.posToCoord(position_3d_eigen.x(),position_3d_eigen.y(),position_3d_eigen.z());

        return voxel_coord;
    }

    Vector3D voxelToPositionCoordinate(const CoordT &voxel_coord)
    {
        auto position_coordinate_default = grid_.coordToPos(voxel_coord);

        Vector3D position_coordinate_required = ConvertPoint<Vector3D>(position_coordinate_default);

        return position_coordinate_required;
    }

    // ========================================================================
    // Statistics
    // ========================================================================
    
    /**
     * @brief Get number of active (allocated) cells
     * @return Active cell count
     * @complexity O(1)
     */
    [[nodiscard]] size_t getActiveCellCount() const noexcept;
    
    /**
     * @brief Get total memory usage in bytes
     * @return Memory usage (includes grid + internal buffers)
     * @complexity O(N) where N = number of chunks
     */
    [[nodiscard]] size_t getMemoryUsage() const noexcept;
    
    /**
     * @brief Get comprehensive statistics
     * @param compute_expensive If true, compute expensive stats (bounding box, distributions)
     * @return Statistics structure
     * @complexity O(1) if !compute_expensive, O(N) if compute_expensive (N = active cells)
     * @performance ~10-50ms for 1M cells when compute_expensive=true
     */
    [[nodiscard]] Occupancy::OccupancyMapStats getStats(bool compute_expensive = false) const;
    
    /**
     * @brief Get quick statistics (no iteration)
     * @return Basic statistics
     * @complexity O(1)
     */
    [[nodiscard]] Occupancy::OccupancyMapStats getQuickStats() const noexcept;
    
    // ========================================================================
    // Queries (const methods)
    // ========================================================================
    
    /**
     * @brief Check if voxel is occupied
     * @param coord Voxel coordinate
     * @return true if probability_log > occupancy_threshold_log
     * @return false if probability_log <= threshold or voxel is unknown
     * @complexity O(log N) where N = number of active voxels
     * @note Thread-safe for concurrent reads (no writes)
     */
    [[nodiscard]] bool isOccupied(const CoordT& coord) const;
    
    /**
     * @brief Check if voxel is unknown (never observed)
     * @param coord Voxel coordinate
     * @return true if voxel never observed OR probability == threshold
     * @complexity O(log N)
     */
    [[nodiscard]] bool isUnknown(const CoordT& coord) const;
    
    /**
     * @brief Check if voxel is free
     * @param coord Voxel coordinate
     * @return true if probability_log < occupancy_threshold_log
     * @return false if probability_log >= threshold or voxel is unknown
     * @complexity O(log N)
     */
    [[nodiscard]] bool isFree(const CoordT& coord) const;
    
    /**
     * @brief Extract all occupied voxel coordinates
     * @param coords Output vector (cleared before filling)
     * @complexity O(N) where N = total active cells
     */
    void getOccupiedVoxels(std::vector<CoordT>& coords) const;
    
    /**
     * @brief Extract all free voxel coordinates
     * @param coords Output vector (cleared before filling)
     * @complexity O(N)
     */
    void getFreeVoxels(std::vector<CoordT>& coords) const;
    
    /**
     * @brief Extract occupied voxels as point cloud
     * @tparam PointT Point type with x,y,z fields (e.g., Eigen::Vector3d, pcl::PointXYZ)
     * @param points Output vector (cleared before filling)
     * @complexity O(N)
     */
    template <typename PointT>
    void getOccupiedVoxels(std::vector<PointT>& points) const {
        thread_local std::vector<CoordT> coords;
        coords.clear();
        
        getOccupiedVoxels(coords);
        points.clear();
        points.reserve(coords.size());
        
        for (const auto& coord : coords) {
            const auto p = grid_.coordToPos(coord);
            points.emplace_back(p.x, p.y, p.z);
        }
    }
    
    /**
     * @brief Extract free voxels as point cloud
     * @tparam PointT Point type with x,y,z fields
     * @param points Output vector (cleared before filling)
     * @complexity O(N)
     */
    template <typename PointT>
    void getFreeVoxels(std::vector<PointT>& points) const {
        thread_local std::vector<CoordT> coords;
        coords.clear();
        
        getFreeVoxels(coords);
        points.clear();
        points.reserve(coords.size());
        
        for (const auto& coord : coords) {
            const auto p = grid_.coordToPos(coord);
            points.emplace_back(p.x, p.y, p.z);
        }
    }

    /**
     * @brief Convert a world-space point into a voxel coordinate using this map's resolution.
     */
    [[nodiscard]] CoordT worldToVoxel(const Vector3D& point) const;

    /**
     * @brief Apply temporal decay to occupied voxels so dynamic obstacles can fade over time.
     *
     * @param dt_seconds Elapsed time since the previous decay update.
     * @param decay_time_seconds Time constant over which occupancy decays toward free-space.
     */
    void applyTemporalDecay(double dt_seconds, double decay_time_seconds);
    
    // ========================================================================
    // Updates (non-const methods)
    // ========================================================================
    
    /**
     * @brief Add a hit observation (occupied voxel)
     * @param point 3D position in map frame
     * @note Uses cached mutable accessor for efficiency
     */
    void addHitPoint(const Vector3D& point);
    
    /**
     * @brief Add a hit observation (occupied voxel)
     * @param coord Voxel coordinate
     * @note Prefer this overload if you already have voxel coordinates
     */
    void addHitPoint(const CoordT& coord);

    /**
     * @brief Reset a voxel to unknown occupancy.
     *
     * Used when a formerly stable obstacle disappears and must be removed
     * from the persistent layer.
     */
    void resetPoint(const CoordT& coord);
    
    /**
     * @brief Add a miss observation (free space)
     * @param point 3D position in map frame
     * @note Uses cached mutable accessor for efficiency
     */
    void addMissPoint(const Vector3D& point);
    
    /**
     * @brief Add a miss observation (free space)
     * @param coord Voxel coordinate
     */
    void addMissPoint(const CoordT& coord);
    
    /**
     * @brief Insert point cloud observation (primary update function)
     * 
     * @tparam PointT Point type with x,y,z fields (e.g., pcl::PointXYZ, Eigen::Vector3d)
     * @tparam Allocator Allocator type for vector
     * 
     * @param points Point cloud in map frame
     * @param origin Sensor position in map frame
     * @param max_range Maximum valid sensor range (meters)
     * 
     * @details For each point:
     *  - If within max_range: mark endpoint as HIT, raytrace from origin as FREE
     *  - If beyond max_range: clip to max_range, mark clipped point as MISS, raytrace as FREE
     * 
     * @note This is the primary update function for SLAM/mapping applications
     * @complexity O(N*R) where N = number of points, R = average ray length
     * @performance ~50-100 µs per point (including raytracing) on modern CPU
     * 
     * @example
     * @code
     * std::vector<Eigen::Vector3d> lidar_points = sensor.getPoints();
     * Eigen::Vector3d sensor_origin = robot.getPose().translation();
     * map.insertPointCloud(lidar_points, sensor_origin, 30.0);
     * @endcode
     */
    template <typename PointT, typename Allocator>
    void insertPointCloud(const std::vector<PointT, Allocator>& points,
                          const PointT& origin,
                          double max_range) {
        const auto from_point = ConvertPoint<Vector3D>(origin);
        const double max_range_sq = max_range * max_range;
        
        // Ensure mutable accessor is valid for entire batch
        ensureAccessorValid();
        
        for (const auto& point : points) {
            const auto to_point = ConvertPoint<Vector3D>(point);
            Vector3D vector_from_to = to_point - from_point;
            const double squared_norm = vector_from_to.squaredNorm();
            
            if (squared_norm >= max_range_sq) {
                // Clip to max range
                Vector3D vector_from_to_unit = vector_from_to / std::sqrt(squared_norm);
                const Vector3D new_to_point = from_point + (vector_from_to_unit * max_range);
                addMissPoint(new_to_point);
            } else {
                addHitPoint(to_point);
            }
        }
        
        updateFreeCells(from_point);
    }
    
    // ========================================================================
    // Memory Management
    // ========================================================================
    
    /**
     * @brief Release unused memory from grid
     * 
     * @warning This invalidates ALL existing accessors (including internal accessors)
     * @post Internal accessors will be automatically recreated on next use
     * 
     * @details Frees leaf grids where all cells are OFF (unobserved/free space).
     *          Also shrinks oversized internal coordinate buffers.
     * 
     * @complexity O(N) where N = number of active chunks
     * @performance Typically 1-10 ms depending on grid size
     * 
     * @note Call this periodically if you observe memory growth from exploration
     */
    void releaseUnusedMemory();
    
    /**
     * @brief Shrink internal coordinate buffers to fit current usage
     * @note Less aggressive than releaseUnusedMemory() - only affects internal buffers
     */
    void shrinkToFit();
    
    // ========================================================================
    // Performance Counters
    // ========================================================================
    
    /**
     * @brief Reset all performance counters to zero
     */
    void resetCounters() noexcept;
    
    // ========================================================================
    // Accessor Management (Advanced)
    // ========================================================================
    
    /**
     * @brief Get mutable accessor (for advanced use)
     * @return Reference to cached mutable accessor
     * @warning Invalidated by releaseUnusedMemory()
     * @note For typical use, prefer addHitPoint/addMissPoint over direct accessor use
     */
    [[nodiscard]] Accessor& getAccessor();
    
    /**
     * @brief Get const accessor (for advanced use)
     * @return ConstAccessor (by value, lightweight)
     * @note Safe to call from const methods
     */
    [[nodiscard]] ConstAccessor getConstAccessor() const;

    void updateFreeCellsPreRayTrace();

private:
    // ========================================================================
    // Internal Methods
    // ========================================================================
    
    void ensureAccessorValid();
    void ensureConstAccessorValid() const;
    void invalidateAccessors();
    void updateFreeCells(const Vector3D& origin);
    void incrementUpdateCount() noexcept;
    
    // ========================================================================
    // Member Variables
    // ========================================================================
    
    // Core data
    VoxelGrid<Occupancy::CellOcc> grid_;
    Occupancy::OccupancyOptions options_;
    
    // State
    uint8_t update_count_{1};                   ///< Update cycle (1→2→3→1...)
    std::vector<CoordT> hit_coords_;            ///< Accumulated hit coordinates
    std::vector<CoordT> miss_coords_;           ///< Accumulated miss coordinates
    
    // Accessor caching (using std::optional for explicit validity)
    std::optional<Accessor> accessor_;                      ///< Mutable accessor
    mutable std::optional<ConstAccessor> const_accessor_;   ///< Const accessor
    
    // Performance counters
    uint64_t total_insertions_{0};              ///< Total point insertions
    uint64_t hit_points_added_{0};              ///< Total hit points
    uint64_t miss_points_added_{0};             ///< Total miss points
};

} // namespace Bonxai
