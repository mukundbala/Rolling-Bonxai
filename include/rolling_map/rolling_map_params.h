#ifndef ROLLING_MAP_PARAMS_H
#define ROLLING_MAP_PARAMS_H
#include <string>
/*
A collection of data structs to
handle passing around data in a 
clean manner
*/
namespace RM
{
struct FrameParams
{
    std::string map_frame {"map"};
    std::string sensor_frame {"laser_scanner"};
    std::string robot_frame {"base_link"};
};

struct MapParams
{
    double resolution {0.1}; //m
    double occupancy_min_threshold {0.12};
    double occupancy_max_threshold {0.97};
};

struct SensorParams
{
    double max_z {3.0}; //m
    double min_z {-3.0}; //m
    double max_range {5.0}; //m
    double min_range {0.5}; //m
    double probability_hit {0.7}; //probability between 0 and 1
    double probability_miss {0.4}; //probability between 0 and 1
};

struct ChunkParams
{
    uint chunk_dim {32}; //int chunk_dim x chunk_dim x chunk_dim
    uint chunk_neib {6}; //6, 18 or 26 cube neigbour
    uint chunk_cache_capacity {7}; //Number of chunks to be in memory
    std::string chunk_folder_path {"chunk_folder"}; //path to the chunk folder
};
}
#endif //ROLLING_MAP_PARAMS_H