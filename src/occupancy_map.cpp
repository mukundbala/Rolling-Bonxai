#include "bonxai_map/occupancy_map.hpp"

namespace Bonxai
{
    [[nodiscard]] int32_t MapUtils::logods(float prob)
    {
        return int32_t(1e6 * std::log(prob / (1.0 - prob)));
    }

    [[nodiscard]] float MapUtils::prob(int32_t logods_fixed)
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

    inline void MapUtils::ComputeRay(const CoordT& key_origin, const CoordT& key_end, std::vector<CoordT>& ray)
    {
        //Clear the ray vector
        ray.clear();
        
        //make the functor
        auto functor = [&ray](const CoordT& coord)
        {
            ray.push_back(coord);
            return true;
        };

        //Get the ray
        RayIterator(key_origin,key_end,functor);
    }

    MapUtils::CellOcc::CellOcc()
    : update_id(4), probability_log(MapUtils::UnknownProbability) {}

    OccupancyMap::OccupancyMap(double resolution)
    :
    grid_(resolution)
    {
        accessor_bound_= false;
    }

    OccupancyMap::OccupancyMap(double resolution , MapUtils::OccupancyOptions& options)
    :
    grid_(resolution),
    options_(options)
    {
        accessor_bound_= false;
    }

    OccupancyMap::OccupancyMap(OccupancyMap&& other) noexcept
    : 
    grid_(std::move(other.grid_))
    {
        accessor_bound_= false;
    }

    OccupancyMap& OccupancyMap::operator=(OccupancyMap&& other) noexcept
    {
        if (this != &other)
        {
            grid_ = std::move(other.grid_);
            accessor_bound_ = false;
        }
        return *this;
    }

    VoxelGrid<MapUtils::CellOcc>& OccupancyMap::getGrid()
    {
        return grid_;
    }

    const VoxelGrid<MapUtils::CellOcc>& OccupancyMap::getConstGrid() const
    {
        return grid_;
    }

    const MapUtils::OccupancyOptions& OccupancyMap::getOptions() const
    {
        return options_;
    }

    void OccupancyMap::setOptions(MapUtils::OccupancyOptions& options)
    {
        this->options_ = options;
    }

    void OccupancyMap::setGrid(VoxelGrid<MapUtils::CellOcc>&& grid)
    {
        this->grid_ = std::move(grid);
        accessor_bound_ = false;
    }

    bool OccupancyMap::isAccessorBound() const
    {
        return this->accessor_bound_;
    }

    void OccupancyMap::setAccessorBound(bool bound)
    {
        this -> accessor_bound_ = bound;
    }

    bool OccupancyMap::isOccupied(const Bonxai::CoordT& coord, Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor) const
    {
        if (auto *cell = accessor.value(coord,false))
        {
            return cell->probability_log > options_.occupancy_threshold_log;
        }

        return false;
    }

    bool OccupancyMap::isUnknown(const Bonxai::CoordT& coord, Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor) const
    {
        if (auto *cell = accessor.value(coord,false))
        {
            return cell->probability_log == options_.occupancy_threshold_log;
        }

        return true;
    }

    bool OccupancyMap::isFree(const Bonxai::CoordT& coord, Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor) const
    {
        if (auto *cell = accessor.value(coord,false))
        {
            return cell->probability_log < options_.occupancy_threshold_log;
        }

        return false;
    }

    void OccupancyMap::getOccupiedVoxels(std::vector<Bonxai::CoordT>& coords) const
    {
        coords.clear();

        //make a visitor functor
        auto visitor = [&](MapUtils::CellOcc &cell,const CoordT& coord)
        {
            if (cell.probability_log > options_.occupancy_threshold_log)
            {
                coords.push_back(coord);
            }
        };

        grid_.forEachCell(visitor);
    }

    void OccupancyMap::getFreeVoxels(std::vector<Bonxai::CoordT>& coords) const
    {
        coords.clear();

        //make a visitor functor
        auto visitor = [&](MapUtils::CellOcc &cell,const CoordT& coord)
        {
            if (cell.probability_log < options_.occupancy_threshold_log)
            {
                coords.push_back(coord);
            }
        };

        grid_.forEachCell(visitor);
    }

    template <typename PointT>
    void OccupancyMap::getOccupiedVoxels(std::vector<PointT>& points)const
    {
        thread_local std::vector<Bonxai::CoordT> coords;
        coords.clear();

        getOccupiedVoxels(coords);
        for (const auto& coord : coords)
        {
            const auto p = grid_.coordToPos(coord);
            points.emplace_back(p.x,p.y,p.z);
        }
    }

    template <typename PointT>
    void OccupancyMap::getFreeVoxels(std::vector<PointT>& points)const
    {
        thread_local std::vector<Bonxai::CoordT> coords;
        coords.clear();

        getFreeVoxels(coords);
        for (const auto& coord : coords)
        {
            const auto p = grid_.coordToPos(coord);
            points.emplace_back(p.x,p.y,p.z);
        }
    }

    void OccupancyMap::addHitPoint(const Vector3D& point,Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor)
    {
        const auto coord = grid_.posToCoord(point);
        MapUtils::CellOcc* cell = accessor.value(coord,true);

        if (cell->update_id != update_count_)
        {
            cell->probability_log = std::min(cell->probability_log + options_.prob_hit_log,
                                            options_.clamp_max_log);
            
            cell->update_id = update_count_;
            hit_coords_.push_back(coord);
        }
    }

    void OccupancyMap::addMissPoint(const Vector3D& point,Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor)
    {
        const auto coord = grid_.posToCoord(point);
        MapUtils::CellOcc* cell = accessor.value(coord,true);

        if (cell->update_id != update_count_)
        {
            cell->probability_log = std::max(cell->probability_log + options_.prob_miss_log,
                                            options_.clamp_min_log);
        }

        cell->update_id = update_count_;
        miss_coords_.push_back(coord);
    }

    template <typename PointT, typename Allocator>
    void OccupancyMap::insertPointCloud(const std::vector<PointT, Allocator>& points, 
                              const PointT& origin, double max_range)
    {
        const auto from_point = ConvertPoint<Vector3D>(origin);
        const double max_range_sq = max_range * max_range;
        auto accessor = grid_.createAccessor();
        for (const auto& point : points)
        {
            const auto to_point = ConvertPoint<Vector3D>(point);
            //For every point, compute the vector from the origin to that point
            Vector3D vector_from_to(to_point - from_point);
            const double squared_norm = vector_from_to.squaredNorm();
            Vector3D vector_from_to_unit = vector_from_to / std::sqrt(squared_norm);

            if (squared_norm >= max_range_sq)
            {
                //Bring the to point to along the radial line r = max_range around from point
                const Vector3D new_to_point = from_point + (vector_from_to_unit * max_range);
                addMissPoint(new_to_point,accessor);
            }
            else
            {
                addHitPoint(to_point,accessor);
            }
        }
        updateFreeCells(from_point);
    }

    void OccupancyMap::updateFreeCells(const Vector3D& origin)
    {
        auto accessor = grid_.createAccessor();

        auto clearPoint = [this,&accessor](const CoordT& coord)
        {
            MapUtils::CellOcc* cell = accessor.value(coord, true);
            if (cell->update_id != update_count_)
            {
                cell->probability_log = std::max(cell->probability_log + options_.prob_miss_log, 
                                                 options_.clamp_min_log);
                cell->update_id = update_count_;
            }
            return true;
        };

        const auto coord_origin = grid_.posToCoord(origin);

        for (const auto& coord_end_hit : hit_coords_)
        {
            MapUtils::RayIterator(coord_origin,coord_end_hit,clearPoint);
        }
        hit_coords_.clear();

        for (const auto& coord_end_miss : miss_coords_)
        {
            MapUtils::RayIterator(coord_origin,coord_end_miss,clearPoint);
        }
        miss_coords_.clear();

        if (++update_count_ == 4)
        {
            update_count_ = 1; //Cycle update count back to 1
        }
    }
}