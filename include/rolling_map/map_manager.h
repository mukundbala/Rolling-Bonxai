#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include "bonxai_map/occupancy_map.hpp"
#include "rolling_map/rolling_map_params.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <vector>
#include <array>
#include <utility>

namespace RM
{
    class MapManager
    {
    public:
        using PCLPoint = pcl::PointXYZ;
        using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;
        using PCLPointCloudSharedPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
        using ChunkKey = std::string;
        using MapPtr = std::shared_ptr<Bonxai::OccupancyMap>;

        /**
         * @brief Constructor for Map Manager class.
         * @param RM::SensorParams sensor_params
         * @param RM::MapParams map_params
         * @param RM::ChunkParams chunk_params
         */
        MapManager(SensorParams& sensor_params,MapParams& map_params,ChunkParams& chunk_params);
        MapManager();

        /**
         * @brief Update function called by some cloud callback to update the entire map
         * @param PCLPointCloud& points
         * @param PCLPoint& origin
         * @return void
         */
        void updateMap(PCLPointCloud& points,PCLPoint& origin);

    private:

        /////////////////////////
        //////Chunk Neibors//////
        /////////////////////////

        /**
         * @brief Get the 6 face neibors of a chunk
         * @param Bonxai::CoordT& c
         * @return std::vector<Bonxai::CoordT>
         */
        std::array<Bonxai::CoordT,6> getFaceNeibors(const Bonxai::CoordT& coord);

        /**
         * @brief Get the 12 edge neibors of a chunk
         * @param Bonxai::CoordT& c
         * @return std::vector<Bonxai::CoordT>
         */
        std::array<Bonxai::CoordT,12> getEdgeNeibors(const Bonxai::CoordT& coord);

        /**
         * @brief Get the 8 corner neibors of a chunk
         * @param Bonxai::CoordT& c
         * @return std::vector<Bonxai::CoordT>
         */
        std::array<Bonxai::CoordT,8> getCornerNeibors(const Bonxai::CoordT& coord);

        /////////////////////////
        //////Conversions////////
        /////////////////////////
        /**
         * @brief Converts a 3D point in map frame to a voxel coord. Exactly the same as Bonxai::posToCoord
         * @param PCLPoint& point
         * @return Bonxai::CoordT
         */
        Bonxai::CoordT mapPointToVoxelCoord(const PCLPoint& point);

        /**
         * @brief Converts a voxel coord to a 3D point in map frame. Exactly the same as Bonxai::coordToPos
         * @param Bonxai::CoordT& coord
         * @return PCLPoint
         */
        PCLPoint voxelCoordToMapPoint(const Bonxai::CoordT& coord);

        /**
         * @brief Convert a voxel coord to a 3D point in the map frame to the center of the voxel
         * @param Bonxai::CoordT& coord
         * @return PCLPoint
         */
        PCLPoint voxelCoordToMapCenterPoint(const Bonxai::CoordT& coord);

        /**
         * @brief COnverts a voxel coord to a chunk coordinate
         * @param Bonxai::CoordT& coord
         * @return CoordT
         */
        Bonxai::CoordT voxelCoordToChunkCoord(const Bonxai::CoordT& coord);

        /**
         * @brief Maps a chunk coordinate to the voxel coordinate, which is the back right down voxel in the map frame
         * @param const Bonxai::CoordT& coord
         * @return Bonxai::CoordT
         */
        Bonxai::CoordT chunkCoordToVoxelCoord(const Bonxai::CoordT& coord);

        /**
         * @brief Maps a 3D point in the map frame to a voxel coordinate that is relative to the voxel origin of the chunk
         * @param const PCLPoint& point
         * @return Bonxai::CoordT
         */
        Bonxai::CoordT mapFramePointToChunkFrameCoord(const PCLPoint& mP);

        /**
         * @brief Maps a chunk coordinate to a chunk key. If coordinate it (x,y,z), then key is x_y_z.
         * @param const Bonxai::CoordT& cc
         * @return ChunkKey
         */
        ChunkKey chunkCoordToChunkKey(const Bonxai::CoordT& cc);

        //Chunk Manager
        //ChunkManager chunk_manager_;

        //parameters
        SensorParams sp_;
        MapParams mp_;
        ChunkParams cp_;
        double inv_resolution_;

        //Check update status
        bool first_update_ {true};

    };

} //namespace RM
#endif // MAP_MANAGER_H