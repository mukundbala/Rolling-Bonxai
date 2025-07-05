#pragma once

#include <eigen3/Eigen/Geometry>
#include <unordered_set>

#include "bonxai_core/bonxai.hpp"


namespace Bonxai
{
    namespace MapUtils
    {
        /**
         * @brief Compute the logds and return as integer
         * @param float probability
         * @return int32_t
         */
        [[nodiscard]] int32_t logods(float prob);

        /**
         * @brief Compute the probability from the logds
         * @param int32_t logods_fixed
         * @return float
         */
        [[nodiscard]] float prob(int32_t logods_fixed);

        /**
         * @brief A Ray iterator that does something for each key on the ray
         * @param CoordT key_origin
         * @param CoordT key_end
         * @param Functor func 
         */
        template <class Functor>
        void RayIterator(const CoordT& key_origin, const CoordT& key_end, const Functor& func);

        /**
         * @brief Compute the ray between two keys
         * @param CoordT key_origin
         * @param CoordT key_end
         * @param std::vector<CoordT>& ray
         */
        inline void ComputeRay(const CoordT& key_origin, const CoordT& key_end, std::vector<CoordT>& ray);

        /**
         * @brief Options for the OccupancyMap
         * @property int32_t prob_miss_log
         * @property int32_t prob_hit_log
         * @property int32_t clamp_min_log
         * @property int32_t clamp_max_log
         * @property int32_t occupancy_threshold_log
         */
        struct OccupancyOptions
        {
            int32_t prob_miss_log = logods(0.4f);
            int32_t prob_hit_log = logods(0.7f);

            int32_t clamp_min_log = logods(0.12f);
            int32_t clamp_max_log = logods(0.97f);

            int32_t occupancy_threshold_log = logods(0.5);
        };

        struct CellOcc
        {
            // variable used to check if a cell was already updated in this loop
            int32_t update_id {4};
            // the probability of the cell to be occupied
            int32_t probability_log {28};

            CellOcc();
        };

        const int32_t UnknownProbability = logods(0.5f);
    }

    class OccupancyMap
    {
    public:
        //Alias for Eigen vectors
        using Vector3D = Eigen::Vector3d;

        /**
         * @brief Constructor
         * @param double resolution
         */
        explicit OccupancyMap(double resolution);

        /**
         * @brief Constructor
         * @param double resolution
         * @param MapUtils::OccupancyOptions& options containing the occupancy options
         */
        explicit OccupancyMap(double resolution, MapUtils::OccupancyOptions& options);

        // Default Destructor
        ~OccupancyMap() = default;

        // NO COPYING! Data structure is super big
        /**
         * @brief Copy constructor. Deleted
         */
        OccupancyMap(const OccupancyMap&) = delete;

        /**
         * @brief Copy Assignment. Deleted
         */
        OccupancyMap& operator=(const OccupancyMap&) = delete;

        // CAN MOVE :)

        /**
         * @brief Move constructor
         * @param OccupancyMap&& other
         */
        OccupancyMap(OccupancyMap&& other) noexcept;

        /**
         * @brief Move Assignment
         * @param OccupancyMap&& other
         * @return OccupancyMap&
         */
        OccupancyMap& operator=(OccupancyMap&& other) noexcept;

        /**
         * @brief Get the Grid object
         * @return VoxelGrid<MapUtils::CellOcc>&
         */
        [[nodiscard]] VoxelGrid<MapUtils::CellOcc>& getGrid();

        /**
         * @brief Get the Grid object
         * @return const VoxelGrid<MapUtils::CellOcc>&
         */
        [[nodiscard]] const VoxelGrid<MapUtils::CellOcc>& getConstGrid() const;


        /**
         * @brief Get the options
         * @return const MapUtils::OccupancyOptions&
         */
        [[nodiscard]] const MapUtils::OccupancyOptions& getOptions() const;

        /**
         * @brief Set the Options object
         * @param MapUtils::OccupancyOptions& options
         */
        void setOptions(MapUtils::OccupancyOptions& options);

        /**
         * @brief Set grid
         * @param VoxelGrid<MapUtils::CellOcc>&& grid
         */
        void setGrid(VoxelGrid<MapUtils::CellOcc>&& grid);

        /**
         * Check if the accessor is bound or not
         * @return bool
         */
        [[nodiscard]] bool isAccessorBound() const;

        /**
         * @brief Set the accessor bound for true, false otherwise
         * @param bool bound
         */
        void setAccessorBound(bool bound);

        
        /**
         * @brief Check if a cell is occupied
         * @param Bonxai::CoordT& coord
         * @return bool
         */
        [[nodiscard]] bool isOccupied(const Bonxai::CoordT& coord, 
                                      Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor) const;

        /**
         * @brief Check if a cell is unknown
         * @param Bonxai::CoordT& coord
         * @return bool
         */
        [[nodiscard]] bool isUnknown(const Bonxai::CoordT& coord, 
                                     Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor) const;

        /**
         * @brief Check if a cell is free
         * @param Bonxai::CoordT& coord
         * @return bool
         */
        [[nodiscard]] bool isFree(const Bonxai::CoordT& coord, 
                                  Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor) const;

        /**
         * @brief Get the occupied voxels where probabality_log > threshold
         * @param std::vector<Bonxai::CoordT>& coords container to hold coords
         */
        void getOccupiedVoxels(std::vector<Bonxai::CoordT>& coords) const;

        /**
         * @brief Get the free voxels where probabality_log < threshold
         * @param std::vector<Bonxai::CoordT>& coords container to hold coords
         */
        void getFreeVoxels(std::vector<Bonxai::CoordT>& coords) const;

        /**
         * @brief Get the occupied voxels templated
         * @param std::vector<Eigen::Vector3d>& points
         */
        template <typename PointT>
        void getOccupiedVoxels(std::vector<PointT>& points) const;

        /**
         * @brief Get the free voxels templated
         * @param std::vector<Eigen::Vector3d>& points
         */
        template <typename PointT>
        void getFreeVoxels(std::vector<PointT>& points) const;

        /**
         * @brief Add points that are hit
         * @param Vector3D& point in map frame
         */
        void addHitPoint(const Vector3D& point,Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor);

        /**
         * @brief Add points that are missed
         * @param Vector3D& point in map frame
         */
        void addMissPoint(const Vector3D& point,Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor);

        /**
         * @brief The main update function exposed to the user to update PointCloud
         * @param std::vector<Eigen::Vector3d>& points containing points on the map
         * @param PointT& origin of the point cloud, which is the map frame position of the camera
         * @param double max_range of the camera
         */
        template <typename PointT, typename Allocator>
        void insertPointCloud(const std::vector<PointT, Allocator>& points, 
                              const PointT& origin, double max_range);
        

    private:
        
        /**
         *@brief Occupancy update for all the cells in the map
         *@param Vector3D& origin
        */
        void updateFreeCells(const Vector3D& origin);
        
        // The main data structure to hold the occupancy grid
        VoxelGrid<MapUtils::CellOcc> grid_;

        // Options for the occupancy grid
        MapUtils::OccupancyOptions options_;
        
        // Update count, cycles from 1 > 2 > 3 > 4 back to 1 and so on
        uint8_t update_count_ = 1;

        // Miss coords for points that are outside the map. Clip to max range. Everything before this is free
        std::vector<CoordT> miss_coords_;

        // Hit coords for terminal points inside the map. Everything before this is free
        std::vector<CoordT> hit_coords_;

        //Accessor to the grid. Generate as little times as possible!
        // mutable Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor accessor_;

        bool accessor_bound_ {false};

        //Maybe there is a way to use STD::optional to manage the lifetime of accessor

    };
}