#pragma once

#include <eigen3/Eigen/Geometry>
#include <unordered_set>

#include "bonxai_core/bonxai.hpp"


namespace Bonxai
{
    namespace OccupancyUtils
    {
        /**
         * @brief Compute the logds and return as integer
         * @param float probability
         * @return int32_t
         */
        [[nodiscard]] constexpr int32_t logods(float prob);
    }
}