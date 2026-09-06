#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#define private public
#include <plan_env/sdf_map.h>
#undef private

namespace {

void initManualTestMap(SDFMap& map) {
  map.mp_.resolution_ = 0.1;
  map.mp_.resolution_inv_ = 10.0;
  map.mp_.map_origin_ = Eigen::Vector3d(-3.0, -3.0, 0.0);
  map.mp_.map_size_ = Eigen::Vector3d(6.0, 6.0, 3.0);
  map.mp_.map_voxel_num_ = Eigen::Vector3i(60, 60, 30);
  map.mp_.map_min_boundary_ = map.mp_.map_origin_;
  map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
  map.mp_.map_min_idx_ = Eigen::Vector3i::Zero();
  map.mp_.map_max_idx_ = map.mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  map.mp_.p_min_ = 0.12;
  map.mp_.p_max_ = 0.90;
  map.mp_.p_occ_ = 0.80;
  map.mp_.clamp_min_log_ = logit(map.mp_.p_min_);
  map.mp_.clamp_max_log_ = logit(map.mp_.p_max_);
  map.mp_.min_occupancy_log_ = logit(map.mp_.p_occ_);
  map.mp_.unknown_flag_ = 0.01;
  map.mp_.ground_height_ = 0.0;
  map.mp_.virtual_ceil_height_ = 2.5;
  map.mp_.visualization_truncate_height_ = 2.49;
  map.mp_.frame_id_ = "world";
  map.mp_.enable_manual_map_ = true;
  map.mp_.manual_click_direct_ = false;
  map.mp_.manual_obstacle_radius_ = 0.35;
  map.mp_.manual_obstacle_height_ = 2.5;
  map.mp_.manual_obstacle_inflate_ = 0.10;
  map.mp_.manual_boundary_padding_ = 0.10;
  map.mp_.manual_boundary_z_min_ = 0.0;
  map.mp_.manual_boundary_z_max_ = 2.5;
  map.mp_.manual_map_auto_load_ = false;
  map.mp_.manual_map_auto_save_ = false;
  map.mp_.manual_map_file_.clear();

  const int buffer_size = map.mp_.map_voxel_num_(0) * map.mp_.map_voxel_num_(1) *
                          map.mp_.map_voxel_num_(2);
  map.md_.occupancy_buffer_ =
      std::vector<double>(buffer_size, map.mp_.clamp_min_log_ - map.mp_.unknown_flag_);
  map.md_.occupancy_buffer_inflate_ = std::vector<char>(buffer_size, 0);
  map.md_.manual_occupancy_buffer_ = std::vector<char>(buffer_size, 0);
  map.md_.distance_buffer_all_ = std::vector<double>(buffer_size, 10000);
  map.md_.manual_boundary_enabled_ = false;
  map.md_.manual_obstacle_centers_.clear();
  map.md_.manual_boundary_points_.clear();
}

geometry_msgs::PointStamped::ConstPtr pointMsg(double x, double y, double z) {
  geometry_msgs::PointStamped::Ptr msg(new geometry_msgs::PointStamped);
  msg->header.frame_id = "world";
  msg->point.x = x;
  msg->point.y = y;
  msg->point.z = z;
  return msg;
}

}  // namespace

TEST(SDFMapManualLayer, ObstacleCallbackMarksManualAndInflatedOccupancy) {
  SDFMap map;
  initManualTestMap(map);

  map.manualObstacleCallback(pointMsg(1.0, 0.0, 1.0));

  Eigen::Vector3d query(1.0, 0.0, 1.0);
  Eigen::Vector3i id;
  map.posToIndex(query, id);
  const int addr = map.toAddress(id);

  EXPECT_EQ(1, map.md_.manual_occupancy_buffer_[addr]);
  EXPECT_EQ(1, map.getInflateOccupancy(query));
  EXPECT_TRUE(map.md_.esdf_need_update_);
  EXPECT_TRUE(map.md_.local_bound_min_ == Eigen::Vector3i::Zero());
  EXPECT_TRUE(map.md_.local_bound_max_ == map.mp_.map_max_idx_);
}

TEST(SDFMapManualLayer, BoundaryUsesTwoPointsBeforeConstrainingMap) {
  SDFMap map;
  initManualTestMap(map);

  map.manualBoundaryCallback(pointMsg(-1.0, -1.0, 1.0));
  EXPECT_FALSE(map.md_.manual_boundary_enabled_);
  EXPECT_TRUE(map.isInMap(Eigen::Vector3d(2.0, 0.0, 1.0)));

  map.manualBoundaryCallback(pointMsg(1.0, 1.0, 1.0));

  EXPECT_TRUE(map.md_.manual_boundary_enabled_);
  EXPECT_TRUE(map.isInMap(Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_FALSE(map.isInMap(Eigen::Vector3d(2.0, 0.0, 1.0)));
  EXPECT_EQ(1, map.getInflateOccupancy(Eigen::Vector3d(-1.05, 0.0, 1.0)));
}

TEST(SDFMapManualLayer, SavesAndLoadsManualObstacleCenters) {
  const std::string file_path = "/tmp/manual_map_layer_test_obstacles.txt";
  std::remove(file_path.c_str());

  {
    SDFMap map;
    initManualTestMap(map);
    map.mp_.manual_map_file_ = file_path;
    map.mp_.manual_map_auto_save_ = true;

    map.manualObstacleCallback(pointMsg(1.0, 0.0, 1.0));
  }

  std::ifstream saved(file_path);
  ASSERT_TRUE(saved.good());
  std::string line;
  std::getline(saved, line);
  EXPECT_FALSE(line.empty());

  SDFMap loaded_map;
  initManualTestMap(loaded_map);
  loaded_map.mp_.manual_map_file_ = file_path;
  loaded_map.mp_.manual_map_auto_load_ = true;

  loaded_map.loadManualMapFile();

  EXPECT_EQ(1, loaded_map.getInflateOccupancy(Eigen::Vector3d(1.0, 0.0, 1.0)));
  ASSERT_EQ(1u, loaded_map.md_.manual_obstacle_centers_.size());
  EXPECT_NEAR(1.0, loaded_map.md_.manual_obstacle_centers_.front().x(), 1e-6);
  EXPECT_NEAR(0.0, loaded_map.md_.manual_obstacle_centers_.front().y(), 1e-6);
  EXPECT_NEAR(1.0, loaded_map.md_.manual_obstacle_centers_.front().z(), 1e-6);

  std::remove(file_path.c_str());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
