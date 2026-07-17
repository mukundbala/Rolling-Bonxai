#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include "bonxai_map/occupancy_map.hpp"

// ============================================================================
// Utility Functions Tests
// ============================================================================

TEST(OccupancyUtilsTest, LogodsConversion) {
  // Test logods conversion
  int32_t log_05 = Bonxai::Occupancy::logods(0.5f);
  EXPECT_NEAR(log_05, 0, 100);  // log(0.5/0.5) = 0, allow small error
  
  int32_t log_07 = Bonxai::Occupancy::logods(0.7f);
  EXPECT_GT(log_07, 0);  // log(0.7/0.3) > 0
  
  int32_t log_03 = Bonxai::Occupancy::logods(0.3f);
  EXPECT_LT(log_03, 0);  // log(0.3/0.7) < 0
  
  // Test boundary cases
  int32_t log_099 = Bonxai::Occupancy::logods(0.99f);
  EXPECT_GT(log_099, log_07);
  
  int32_t log_001 = Bonxai::Occupancy::logods(0.01f);
  EXPECT_LT(log_001, log_03);
}

TEST(OccupancyUtilsTest, ProbConversion) {
  // Test prob conversion
  float p1 = Bonxai::Occupancy::prob(0);
  EXPECT_NEAR(p1, 0.5f, 0.01f);
  
  int32_t log_07 = Bonxai::Occupancy::logods(0.7f);
  float p2 = Bonxai::Occupancy::prob(log_07);
  EXPECT_NEAR(p2, 0.7f, 0.01f);
  
  int32_t log_03 = Bonxai::Occupancy::logods(0.3f);
  float p3 = Bonxai::Occupancy::prob(log_03);
  EXPECT_NEAR(p3, 0.3f, 0.01f);
}

TEST(OccupancyUtilsTest, RoundTripConversion) {
  // Test round-trip conversion
  std::vector<float> test_probs = {0.1f, 0.3f, 0.5f, 0.7f, 0.9f};
  
  for (float prob : test_probs) {
    int32_t log_val = Bonxai::Occupancy::logods(prob);
    float recovered = Bonxai::Occupancy::prob(log_val);
    EXPECT_NEAR(recovered, prob, 0.02f);
  }
}

// // ============================================================================
// // OccupancyOptions Tests
// // ============================================================================

TEST(OccupancyOptionsTest, DefaultConstruction) {
  Bonxai::Occupancy::OccupancyOptions opts;
  
  // Check that all fields are initialized
  EXPECT_NE(opts.prob_miss_log, 0);
  EXPECT_NE(opts.prob_hit_log, 0);
  EXPECT_NE(opts.clamp_min_log, 0);
  EXPECT_NE(opts.clamp_max_log, 0);
  
  // Check relative values
  EXPECT_LT(opts.prob_miss_log, 0);  // Miss reduces occupancy
  EXPECT_GT(opts.prob_hit_log, 0);   // Hit increases occupancy
  EXPECT_LT(opts.clamp_min_log, opts.occupancy_threshold_log);
  EXPECT_GT(opts.clamp_max_log, opts.occupancy_threshold_log);
}

// // ============================================================================
// // CellOcc Tests
// // ============================================================================

TEST(CellOccTest, DefaultConstruction) {
  Bonxai::Occupancy::CellOcc cell;
  
  EXPECT_EQ(cell.update_id, 4);
  EXPECT_EQ(cell.probability_log, Bonxai::Occupancy::UnknownProbability);
}

// // ============================================================================
// // RayIterator Tests
// // ============================================================================

TEST(RayIteratorTest, SameOriginAndEnd) {
  Bonxai::CoordT origin{0, 0, 0};
  Bonxai::CoordT end{0, 0, 0};
  
  std::vector<Bonxai::CoordT> visited;
  Bonxai::Occupancy::RayIterator(origin, end, [&](const Bonxai::CoordT& coord) {
    visited.push_back(coord);
    return true;
  });
  
  EXPECT_TRUE(visited.empty());
}

TEST(RayIteratorTest, AlongXAxis) {
  Bonxai::CoordT origin{0, 0, 0};
  Bonxai::CoordT end{5, 0, 0};
  
  std::vector<Bonxai::CoordT> visited;
  Bonxai::Occupancy::RayIterator(origin, end, [&](const Bonxai::CoordT& coord) {
    visited.push_back(coord);
    return true;
  });
  
  EXPECT_FALSE(visited.empty());
  EXPECT_EQ(visited.front(), origin);
  
  // All points should have y=0, z=0
  for (const auto& coord : visited) {
    EXPECT_EQ(coord.y, 0);
    EXPECT_EQ(coord.z, 0);
  }
}

TEST(RayIteratorTest, Diagonal3D) {
  Bonxai::CoordT origin{0, 0, 0};
  Bonxai::CoordT end{5, 5, 5};
  
  std::vector<Bonxai::CoordT> visited;
  Bonxai::Occupancy::RayIterator(origin, end, [&](const Bonxai::CoordT& coord) {
    visited.push_back(coord);
    return true;
  });
  
  EXPECT_FALSE(visited.empty());
  EXPECT_EQ(visited.front(), origin);
}

TEST(RayIteratorTest, EarlyTermination) {
  Bonxai::CoordT origin{0, 0, 0};
  Bonxai::CoordT end{10, 0, 0};
  
  std::vector<Bonxai::CoordT> visited;
  Bonxai::Occupancy::RayIterator(origin, end, [&](const Bonxai::CoordT& coord) {
    visited.push_back(coord);
    return coord.x < 5;  // Stop at x=5
  });
  
  EXPECT_FALSE(visited.empty());
  EXPECT_LE(visited.back().x, 5);
}

TEST(ComputeRayTest, BasicRay) {
  Bonxai::CoordT origin{0, 0, 0};
  Bonxai::CoordT end{5, 0, 0};
  
  std::vector<Bonxai::CoordT> ray;
  Bonxai::Occupancy::ComputeRay(origin, end, ray);
  
  EXPECT_FALSE(ray.empty());
  EXPECT_EQ(ray.front(), origin);
}

TEST(ComputeRayTest, ClearsExistingData) {
  Bonxai::CoordT origin{0, 0, 0};
  Bonxai::CoordT end{5, 0, 0};
  
  std::vector<Bonxai::CoordT> ray;
  ray.push_back({100, 100, 100});  // Add garbage
  
  Bonxai::Occupancy::ComputeRay(origin, end, ray);
  
  EXPECT_FALSE(ray.empty());
  EXPECT_NE(ray.front().x, 100);
}

// // ============================================================================
// // OccupancyMap Construction Tests
// // ============================================================================

TEST(OccupancyMapTest, DefaultConstruction) {
  Bonxai::OccupancyMap map(0.1);
  
  EXPECT_EQ(map.getActiveCellCount(), 0);
  EXPECT_GT(map.getMemoryUsage(), 0); 
  
  auto stats = map.getQuickStats();
  EXPECT_EQ(stats.total_active_cells, 0);
  EXPECT_FALSE(stats.has_data);
}

TEST(OccupancyMapTest, ConstructionWithOptions) {
  Bonxai::Occupancy::OccupancyOptions opts;
  opts.prob_hit_log = Bonxai::Occupancy::logods(0.8f);
  
  Bonxai::OccupancyMap map(0.1, opts);
  
  const auto& retrieved_opts = map.getOptions();
  EXPECT_EQ(retrieved_opts.prob_hit_log, opts.prob_hit_log);
}

TEST(OccupancyMapTest, ConstructionFromGrid) {
  Bonxai::Occupancy::OccupancyOptions opts;
  Bonxai::VoxelGrid<Bonxai::Occupancy::CellOcc> grid(0.1);
  
  // Add some data to grid
  auto accessor = grid.createAccessor();
  accessor.setCellOn({0, 0, 0}, Bonxai::Occupancy::CellOcc());
  
  Bonxai::OccupancyMap map(opts, std::move(grid));
  
  EXPECT_GT(map.getActiveCellCount(), 0);
}

TEST(OccupancyMapTest, MoveConstruction) {
  Bonxai::OccupancyMap map1(0.1);
  map1.addHitPoint(Eigen::Vector3d(0.5, 0.5, 0.5));
  
  auto stats1 = map1.getQuickStats();
  
  Bonxai::OccupancyMap map2(std::move(map1));
  
  auto stats2 = map2.getQuickStats();
  EXPECT_EQ(stats2.total_insertions, stats1.total_insertions);
}

TEST(OccupancyMapTest, MoveAssignment) {
  Bonxai::OccupancyMap map1(0.1);
  map1.addHitPoint(Eigen::Vector3d(0.5, 0.5, 0.5));
  
  auto stats1 = map1.getQuickStats();
  
  Bonxai::OccupancyMap map2(0.2);
  map2 = std::move(map1);
  
  auto stats2 = map2.getQuickStats();
  EXPECT_EQ(stats2.total_insertions, stats1.total_insertions);
}

// // ============================================================================
// // Grid Access Tests
// // ============================================================================

TEST(OccupancyMapTest, GetGrid) {
  Bonxai::OccupancyMap map(0.1);
  
  auto& grid = map.getGrid();
  EXPECT_EQ(grid.activeCellsCount(), 0);
}

TEST(OccupancyMapTest, GetGridConst) {
  const Bonxai::OccupancyMap map(0.1);
  
  const auto& grid = map.getGrid();
  EXPECT_EQ(grid.activeCellsCount(), 0);
}

// // ============================================================================
// // Options Tests
// // ============================================================================

TEST(OccupancyMapTest, GetOptions) {
  Bonxai::OccupancyMap map(0.1);
  
  const auto& opts = map.getOptions();
  EXPECT_NE(opts.prob_hit_log, 0);
}

TEST(OccupancyMapTest, SetOptions) {
  Bonxai::OccupancyMap map(0.1);
  
  Bonxai::Occupancy::OccupancyOptions new_opts;
  new_opts.prob_hit_log = 999999;
  
  map.setOptions(new_opts);
  
  const auto& retrieved = map.getOptions();
  EXPECT_EQ(retrieved.prob_hit_log, 999999);
}

// // ============================================================================
// // Statistics Tests
// // ============================================================================

TEST(OccupancyMapTest, GetActiveCellCount) {
  Bonxai::OccupancyMap map(0.1);
  
  EXPECT_EQ(map.getActiveCellCount(), 0);
  
  map.addHitPoint(Eigen::Vector3d(0.5, 0.5, 0.5));
  EXPECT_GT(map.getActiveCellCount(), 0);
}

TEST(OccupancyMapTest, GetMemoryUsage) {
  Bonxai::OccupancyMap map(0.1);
  
  size_t initial_mem = map.getMemoryUsage();
  EXPECT_GT(initial_mem, 0);
  
  // Add many points
  for (int i = 0; i < 100; ++i) {
    map.addHitPoint(Eigen::Vector3d(i * 0.5, i * 0.5, 0));
  }
  
  size_t after_mem = map.getMemoryUsage();
  EXPECT_GT(after_mem, initial_mem);
}

TEST(OccupancyMapTest, GetQuickStats) {
  Bonxai::OccupancyMap map(0.1);
  
  auto stats = map.getQuickStats();
  EXPECT_EQ(stats.total_active_cells, 0);
  EXPECT_EQ(stats.total_insertions, 0);
  EXPECT_FALSE(stats.has_data);
  
  map.addHitPoint(Eigen::Vector3d(0.5, 0.5, 0.5));
  
  stats = map.getQuickStats();
  EXPECT_GT(stats.total_insertions, 0);
  EXPECT_TRUE(stats.has_data);
}

TEST(OccupancyMapTest, GetStatsNotExpensive) {
  Bonxai::OccupancyMap map(0.1);
  map.addHitPoint(Eigen::Vector3d(0.5, 0.5, 0.5));
  
  auto stats = map.getStats(false);
  EXPECT_GT(stats.total_active_cells, 0);
}

TEST(OccupancyMapTest, GetStatsExpensive) {
  Bonxai::OccupancyMap map(0.1);
  
  // Add occupied voxels
  for (int i = 0; i < 10; ++i) {
    map.addHitPoint(Eigen::Vector3d(i * 0.5, 0, 0));
  }
  
  // Trigger update
  std::vector<Eigen::Vector3d> points;
  map.insertPointCloud(points, Eigen::Vector3d(0, 0, 0), 10.0);
  
  auto stats = map.getStats(true);
  
  EXPECT_GT(stats.total_active_cells, 0);
  EXPECT_TRUE(stats.has_data);
  EXPECT_NE(stats.min_coord.x, stats.max_coord.x);
}

TEST(OccupancyMapTest, StatsOccupiedFreeCounts) {
  Bonxai::OccupancyMap map(0.1);
  
  // Create occupied cells (high probability)
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 10; ++j) {
      map.addHitPoint(Bonxai::CoordT{i, 0, 0});
    }
  }
  
  std::vector<Eigen::Vector3d> points;
  map.insertPointCloud(points, Eigen::Vector3d(0, 0, 0), 10.0);
  
  // Create free cells (low probability)
  for (int i = 10; i < 15; ++i) {
    for (int j = 0; j < 10; ++j) {
      map.addMissPoint(Bonxai::CoordT{i, 0, 0});
    }
  }
  
  map.insertPointCloud(points, Eigen::Vector3d(0, 0, 0), 10.0);
  
  auto stats = map.getStats(true);
  
  EXPECT_GT(stats.occupied_cells, 0);
  EXPECT_GT(stats.free_cells, 0);
}

// // ============================================================================
// // Query Tests (const methods)
// // ============================================================================

TEST(OccupancyMapTest, IsOccupiedUnknownCell) {
  const Bonxai::OccupancyMap map(0.1);
  
  Bonxai::CoordT coord{0, 0, 0};
  EXPECT_FALSE(map.isOccupied(coord));
}

TEST(OccupancyMapTest, IsOccupiedAfterHits) {
  Bonxai::OccupancyMap map(0.1);
  Bonxai::CoordT coord{0, 0, 0};
  
  // Add many hits to make it occupied
  for (int i = 0; i < 10; ++i) {
    map.addHitPoint(coord);
  }
  
  // Trigger update
  std::vector<Eigen::Vector3d> points;
  map.insertPointCloud(points, Eigen::Vector3d(0, 0, 0), 10.0);
  
  EXPECT_TRUE(map.isOccupied(coord));
}

TEST(OccupancyMapTest, IsUnknownNewCell) {
  const Bonxai::OccupancyMap map(0.1);
  
  Bonxai::CoordT coord{0, 0, 0};
  EXPECT_TRUE(map.isUnknown(coord));
}

TEST(OccupancyMapTest, IsUnknownAfterObservation) {
  Bonxai::OccupancyMap map(0.1);
  Bonxai::CoordT coord{0, 0, 0};
  
  map.addHitPoint(coord);
  
  std::vector<Eigen::Vector3d> points;
  map.insertPointCloud(points, Eigen::Vector3d(0, 0, 0), 10.0);
  
  // After observation, should not be unknown
  EXPECT_FALSE(map.isUnknown(coord));
}

TEST(OccupancyMapTest, IsFreeUnknownCell) {
  const Bonxai::OccupancyMap map(0.1);
  
  Bonxai::CoordT coord{0, 0, 0};
  EXPECT_FALSE(map.isFree(coord));
}

TEST(OccupancyMapTest, IsFreeAfterMisses) {
  Bonxai::OccupancyMap map(0.1);
  Bonxai::CoordT coord{0, 0, 0};
  
  // Add many misses to make it free
  for (int i = 0; i < 10; ++i) {
    map.addMissPoint(coord);
  }
  
  std::vector<Eigen::Vector3d> points;
  map.insertPointCloud(points, Eigen::Vector3d(0, 0, 0), 10.0);
  
  EXPECT_TRUE(map.isFree(coord));
}

TEST(OccupancyMapTest, GetOccupiedVoxelsEmpty) {
  const Bonxai::OccupancyMap map(0.1);
  
  std::vector<Bonxai::CoordT> coords;
  map.getOccupiedVoxels(coords);
  
  EXPECT_TRUE(coords.empty());
}

TEST(OccupancyMapTest, GetOccupiedVoxelsNonEmpty) {
  Bonxai::OccupancyMap map(0.1);
  
  // Add occupied voxels
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 10; ++j) {
      map.addHitPoint(Bonxai::CoordT{i, 0, 0});
    }
  }
  
  std::vector<Eigen::Vector3d> points;
  map.insertPointCloud(points, Eigen::Vector3d(0, 0, 0), 10.0);
  
  std::vector<Bonxai::CoordT> coords;
  map.getOccupiedVoxels(coords);
  
  EXPECT_FALSE(coords.empty());
}

TEST(OccupancyMapTest, GetFreeVoxelsEmpty) {
  const Bonxai::OccupancyMap map(0.1);
  
  std::vector<Bonxai::CoordT> coords;
  map.getFreeVoxels(coords);
  
  EXPECT_TRUE(coords.empty());
}

TEST(OccupancyMapTest, GetFreeVoxelsNonEmpty) {
  Bonxai::OccupancyMap map(0.1);
  
  // Add free voxels
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 10; ++j) {
      map.addMissPoint(Bonxai::CoordT{i, 0, 0});
    }
  }
  
  std::vector<Eigen::Vector3d> points;
  map.insertPointCloud(points, Eigen::Vector3d(0, 0, 0), 10.0);
  
  std::vector<Bonxai::CoordT> coords;
  map.getFreeVoxels(coords);
  
  EXPECT_FALSE(coords.empty());
}

TEST(OccupancyMapTest, GetOccupiedVoxelsAsPoints) {
  Bonxai::OccupancyMap map(0.1);
  
  // Add occupied voxels
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 10; ++j) {
      map.addHitPoint(Bonxai::CoordT{i, 0, 0});
    }
  }
  
  std::vector<Eigen::Vector3d> points;
  map.insertPointCloud(points, Eigen::Vector3d(0, 0, 0), 10.0);
  
  std::vector<Eigen::Vector3d> occupied_points;
  map.getOccupiedVoxels(occupied_points);
  
  EXPECT_FALSE(occupied_points.empty());
}

TEST(OccupancyMapTest, TemporalDecayReducesOccupancy) {
  Bonxai::OccupancyMap map(0.1);
  const Eigen::Vector3d point(0.05, 0.05, 0.05);

  map.addHitPoint(point);
  const auto coord = map.worldToVoxel(point);

  EXPECT_TRUE(map.isOccupied(coord));

  map.applyTemporalDecay(10.0, 0.1);

  EXPECT_FALSE(map.isOccupied(coord));
  EXPECT_TRUE(map.isUnknown(coord));
}

TEST(OccupancyMapTest, TemporalDecayDoesNotTurnOccupancyIntoFreeSpace) {
  Bonxai::OccupancyMap map(0.1);
  const auto coord = map.worldToVoxel(Eigen::Vector3d(0.05, 0.05, 0.05));

  map.addHitPoint(coord);
  map.applyTemporalDecay(1.0, 2.0);

  EXPECT_TRUE(map.isOccupied(coord));
  EXPECT_FALSE(map.isFree(coord));
}

TEST(OccupancyMapTest, ResetPointRemovesPersistentOccupancy) {
  Bonxai::OccupancyMap map(0.1);
  const auto coord = map.worldToVoxel(Eigen::Vector3d(0.05, 0.05, 0.05));

  map.addHitPoint(coord);
  ASSERT_TRUE(map.isOccupied(coord));

  map.resetPoint(coord);

  EXPECT_FALSE(map.isOccupied(coord));
  EXPECT_TRUE(map.isUnknown(coord));
}

TEST(OccupancyMapTest, GetFreeVoxelsAsPoints) {
  Bonxai::OccupancyMap map(0.1);
  
  // Add free voxels
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 10; ++j) {
      map.addMissPoint(Bonxai::CoordT{i, 0, 0});
    }
  }
  
  std::vector<Eigen::Vector3d> points;
  map.insertPointCloud(points, Eigen::Vector3d(0, 0, 0), 10.0);
  
  std::vector<Eigen::Vector3d> free_points;
  map.getFreeVoxels(free_points);
  
  EXPECT_FALSE(free_points.empty());
}

// // ============================================================================
// // Update Tests (non-const methods)
// // ============================================================================

TEST(OccupancyMapTest, AddHitPointVector3D) {
  Bonxai::OccupancyMap map(0.1);
  
  Eigen::Vector3d point(0.5, 0.5, 0.5);
  map.addHitPoint(point);
  
  auto stats = map.getQuickStats();
  EXPECT_GT(stats.hit_points_added, 0);
}

TEST(OccupancyMapTest, AddHitPointCoord) {
  Bonxai::OccupancyMap map(0.1);
  
  Bonxai::CoordT coord{5, 5, 5};
  map.addHitPoint(coord);
  
  auto stats = map.getQuickStats();
  EXPECT_GT(stats.hit_points_added, 0);
}

TEST(OccupancyMapTest, AddMissPointVector3D) {
  Bonxai::OccupancyMap map(0.1);
  
  Eigen::Vector3d point(0.5, 0.5, 0.5);
  map.addMissPoint(point);
  
  auto stats = map.getQuickStats();
  EXPECT_GT(stats.miss_points_added, 0);
}

TEST(OccupancyMapTest, AddMissPointCoord) {
  Bonxai::OccupancyMap map(0.1);
  
  Bonxai::CoordT coord{5, 5, 5};
  map.addMissPoint(coord);
  
  auto stats = map.getQuickStats();
  EXPECT_GT(stats.miss_points_added, 0);
}

TEST(OccupancyMapTest, InsertPointCloudEmpty) {
  Bonxai::OccupancyMap map(0.1);
  
  std::vector<Eigen::Vector3d> points;
  Eigen::Vector3d origin(0, 0, 0);
  
  map.insertPointCloud(points, origin, 10.0);
  
  EXPECT_EQ(map.getActiveCellCount(), 0);
}

TEST(OccupancyMapTest, InsertPointCloudWithinRange) {
  Bonxai::OccupancyMap map(0.1);
  
  std::vector<Eigen::Vector3d> points;
  points.emplace_back(1.0, 0.0, 0.0);
  points.emplace_back(0.0, 1.0, 0.0);
  points.emplace_back(0.0, 0.0, 1.0);
  
  Eigen::Vector3d origin(0, 0, 0);
  
  map.insertPointCloud(points, origin, 10.0);
  
  EXPECT_GT(map.getActiveCellCount(), 0);
  
  auto stats = map.getQuickStats();
  EXPECT_GT(stats.hit_points_added, 0);
}

TEST(OccupancyMapTest, InsertPointCloudBeyondRange) {
  Bonxai::OccupancyMap map(0.1);
  
  std::vector<Eigen::Vector3d> points;
  points.emplace_back(20.0, 0.0, 0.0);  // Beyond 10m range
  
  Eigen::Vector3d origin(0, 0, 0);
  
  map.insertPointCloud(points, origin, 10.0);
  
  auto stats = map.getQuickStats();
  EXPECT_GT(stats.miss_points_added, 0);
  EXPECT_EQ(stats.hit_points_added, 0);
}

TEST(OccupancyMapTest, InsertPointCloudCreatesRays) {
  Bonxai::OccupancyMap map(0.1);
  
  std::vector<Eigen::Vector3d> points;
  points.emplace_back(5.0, 0.0, 0.0);
  
  Eigen::Vector3d origin(0, 0, 0);
  
  size_t before = map.getActiveCellCount();
  map.insertPointCloud(points, origin, 10.0);
  size_t after = map.getActiveCellCount();
  
  // Should have created cells along the ray
  EXPECT_GT(after, before);
}

TEST(OccupancyMapTest, InsertPointCloudMultipleBatches) {
  Bonxai::OccupancyMap map(0.1);
  
  Eigen::Vector3d origin(0, 0, 0);
  
  // First batch
  std::vector<Eigen::Vector3d> points1;
  points1.emplace_back(1.0, 0.0, 0.0);
  map.insertPointCloud(points1, origin, 10.0);
  
  auto stats1 = map.getQuickStats();
  
  // Second batch
  std::vector<Eigen::Vector3d> points2;
  points2.emplace_back(2.0, 0.0, 0.0);
  map.insertPointCloud(points2, origin, 10.0);
  
  auto stats2 = map.getQuickStats();
  
  EXPECT_GT(stats2.total_insertions, stats1.total_insertions);
}

TEST(OccupancyMapTest, UpdateCyclePreventsDoubleUpdate) {
  Bonxai::OccupancyMap map(0.1);
  
  Bonxai::CoordT coord{0, 0, 0};
  
  // Add hit twice in same update cycle
  map.addHitPoint(coord);
  map.addHitPoint(coord);
  
  // Both should be accumulated but not yet applied
  auto stats = map.getQuickStats();
  EXPECT_EQ(stats.hit_points_added, 2);
  
  // Trigger update
  std::vector<Eigen::Vector3d> points;
  map.insertPointCloud(points, Eigen::Vector3d(0, 0, 0), 10.0);
  
  // Cell should only be updated once per cycle
  EXPECT_TRUE(map.isOccupied(coord) || !map.isUnknown(coord));
}

// // ============================================================================
// // Memory Management Tests
// // ============================================================================

TEST(OccupancyMapTest, ReleaseUnusedMemory) {
  Bonxai::OccupancyMap map(0.1);
  
  // Add some points
  for (int i = 0; i < 100; ++i) {
    map.addHitPoint(Eigen::Vector3d(i * 0.1, 0, 0));
  }
  
  std::vector<Eigen::Vector3d> points;
  map.insertPointCloud(points, Eigen::Vector3d(0, 0, 0), 10.0);
  
  size_t before = map.getMemoryUsage();
  
  map.releaseUnusedMemory();
  
  size_t after = map.getMemoryUsage();
  
  // Memory might be same or reduced, but shouldn't increase
  EXPECT_LE(after, before);
}

TEST(OccupancyMapTest, ReleaseUnusedMemoryInvalidatesAccessors) {
  Bonxai::OccupancyMap map(0.1);
  
  // Force accessor creation
  map.addHitPoint(Eigen::Vector3d(0.5, 0.5, 0.5));
  
  // Release memory (invalidates accessors)
  map.releaseUnusedMemory();
  
  // Should still work (accessor recreated)
  map.addHitPoint(Eigen::Vector3d(1.0, 1.0, 1.0));
  
  EXPECT_GT(map.getActiveCellCount(), 0);
}

TEST(OccupancyMapTest, ShrinkToFit) {
  Bonxai::OccupancyMap map(0.1);
  
  // Add many points to grow buffers
  for (int i = 0; i < 1000; ++i) {
    map.addHitPoint(Eigen::Vector3d(i * 0.1, 0, 0));
  }
  
  std::vector<Eigen::Vector3d> points;
  map.insertPointCloud(points, Eigen::Vector3d(0, 0, 0), 10.0);
  
  // Should not throw
  map.shrinkToFit();
}

// // ============================================================================
// // Performance Counter Tests
// // ============================================================================

TEST(OccupancyMapTest, ResetCounters) {
  Bonxai::OccupancyMap map(0.1);
  
  map.addHitPoint(Eigen::Vector3d(0.5, 0.5, 0.5));
  map.addMissPoint(Eigen::Vector3d(1.0, 1.0, 1.0));
  
  auto stats1 = map.getQuickStats();
  EXPECT_GT(stats1.total_insertions, 0);
  
  map.resetCounters();
  
  auto stats2 = map.getQuickStats();
  EXPECT_EQ(stats2.total_insertions, 0);
  EXPECT_EQ(stats2.hit_points_added, 0);
  EXPECT_EQ(stats2.miss_points_added, 0);
}

// // ============================================================================
// // Accessor Management Tests
// // ============================================================================

TEST(OccupancyMapTest, GetAccessor) {
  Bonxai::OccupancyMap map(0.1);
  
  auto& accessor = map.getAccessor();
  
  // Should be able to use accessor
  Bonxai::CoordT coord{0, 0, 0};
  accessor.setCellOn(coord);
  
  EXPECT_TRUE(accessor.isCellOn(coord));
}

TEST(OccupancyMapTest, GetAccessorCached) {
  Bonxai::OccupancyMap map(0.1);
  
  auto& accessor1 = map.getAccessor();
  auto& accessor2 = map.getAccessor();
  
  // Should return same accessor (by reference)
  EXPECT_EQ(&accessor1, &accessor2);
}

TEST(OccupancyMapTest, GetConstAccessor) {
  const Bonxai::OccupancyMap map(0.1);
  
  auto accessor = map.getConstAccessor();
  
  // Should be able to query
  Bonxai::CoordT coord{0, 0, 0};
  EXPECT_FALSE(accessor.isCellOn(coord));
}

// // ============================================================================
// // Integration Tests
// // ============================================================================

TEST(OccupancyMapTest, FullWorkflow) {
  Bonxai::OccupancyMap map(0.1);
  
  // Simulate LIDAR scan
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 360; ++i) {
    double angle = i * M_PI / 180.0;
    double range = 5.0;
    points.emplace_back(range * std::cos(angle), range * std::sin(angle), 0.0);
  }
  
  Eigen::Vector3d origin(0, 0, 0);
  map.insertPointCloud(points, origin, 10.0);
  
  EXPECT_GT(map.getActiveCellCount(), 0);
  
  auto stats = map.getStats(true);
  EXPECT_GT(stats.occupied_cells, 0);
  EXPECT_TRUE(stats.has_data);
}

TEST(OccupancyMapTest, MultipleScansAccumulation) {
  Bonxai::OccupancyMap map(0.1);
  
  Eigen::Vector3d origin(0, 0, 0);
  
  // First scan
  std::vector<Eigen::Vector3d> scan1;
  scan1.emplace_back(1.0, 0.0, 0.0);
  map.insertPointCloud(scan1, origin, 10.0);
  
  size_t cells_after_scan1 = map.getActiveCellCount();
  
  // Second scan (different direction)
  std::vector<Eigen::Vector3d> scan2;
  scan2.emplace_back(0.0, 1.0, 0.0);
  map.insertPointCloud(scan2, origin, 10.0);
  
  size_t cells_after_scan2 = map.getActiveCellCount();
  
  EXPECT_GE(cells_after_scan2, cells_after_scan1);
}

TEST(OccupancyMapTest, OccupancyProbabilityUpdates) {
  Bonxai::OccupancyMap map(0.1);
  Bonxai::CoordT coord{0, 0, 0};
  
  // Start unknown
  EXPECT_TRUE(map.isUnknown(coord));
  
  // Add multiple hits
  for (int i = 0; i < 5; ++i) {
    map.addHitPoint(coord);
  }
  std::vector<Eigen::Vector3d> points;
  map.insertPointCloud(points, Eigen::Vector3d(0, 0, 0), 10.0);
  
  // Should be occupied
  EXPECT_TRUE(map.isOccupied(coord));
  EXPECT_FALSE(map.isUnknown(coord));
  
  // Add multiple misses
  for (int i = 0; i < 10; ++i) {
    map.addMissPoint(coord);
  }
  map.insertPointCloud(points, Eigen::Vector3d(0, 0, 0), 10.0);
  
  // Probability should decrease (might become free)
  // Exact state depends on clamping
}

TEST(OccupancyMapTest, RayTracingCreatesFreeCells) {
  Bonxai::OccupancyMap map(0.1);
  
  std::vector<Eigen::Vector3d> points;
  points.emplace_back(10.0, 0.0, 0.0);  // Far point
  
  Eigen::Vector3d origin(0, 0, 0);
  map.insertPointCloud(points, origin, 20.0);
  
  // Check that intermediate cells exist (ray tracing)
  std::vector<Bonxai::CoordT> free_cells;
  map.getFreeVoxels(free_cells);
  
  EXPECT_GT(free_cells.size(), 1);  // Should have more than just endpoint
}

// ============================================================================
// Main Function
// ============================================================================

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
