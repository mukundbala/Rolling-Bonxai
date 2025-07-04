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
        [[nodiscard]] constexpr int32_t logods(float prob);

        /**
         * @brief Compute the probability from the logds
         * @param int32_t logods_fixed
         * @return float
         */
        [[nodiscard]] constexpr float prob(int32_t logods_fixed);

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
}