#include "bonxai_map/occupancy_map.hpp"


namespace Bonxai
{
    [[nodiscard]] constexpr int32_t MapUtils::logods(float prob)
    {
        return int32_t(1e6 * std::log(prob / (1.0 - prob)));
    }

    [[nodiscard]] constexpr float MapUtils::prob(int32_t logods_fixed)
    {
        float logods = float(logods_fixed) * 1e-6;
        return (1.0 - 1.0 / (1.0 + std::exp(logods)));
    }

    template <class Functor>
    inline void MapUtils::RayIterator(const CoordT& key_origin, const CoordT& key_end, const Functor& func) 
    {
        if (key_origin == key_end) {return;}
        if (!func(key_origin)) {return;}

        CoordT error = {0, 0, 0};
        CoordT coord = key_origin;
        CoordT delta = (key_end - coord);
        const CoordT step = {delta.x < 0 ? -1 : 1, delta.y < 0 ? -1 : 1, delta.z < 0 ? -1 : 1};

        delta = {
            delta.x < 0 ? -delta.x : delta.x, delta.y < 0 ? -delta.y : delta.y,
            delta.z < 0 ? -delta.z : delta.z};

        const int max = std::max(std::max(delta.x, delta.y), delta.z);

        // maximum change of any coordinate
        for (int i = 0; i < max - 1; ++i) 
        {
            // update errors
            error = error + delta;
            // manual loop unrolling
            if ((error.x << 1) >= max) 
            {
                coord.x += step.x;
                error.x -= max;
            }

            if ((error.y << 1) >= max) 
            {
                coord.y += step.y;
                error.y -= max;
            }

            if ((error.z << 1) >= max) 
            {
                coord.z += step.z;
                error.z -= max;
            }

            if (!func(coord)) {return;}
        }
    }

    MapUtils::CellOcc::CellOcc()
    : update_id(4), probability_log(MapUtils::UnknownProbability) {}

}