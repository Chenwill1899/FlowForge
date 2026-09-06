#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <map_generator/SaveScene.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <functional>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

constexpr char kWorldFrame[] = "world";
constexpr double kMaximumCoordinate = 100000.0;
constexpr long double kMaximumSceneSamples = 20000000.0L;

bool isFinite(double value) { return std::isfinite(value); }

bool isAcceptedFrame(const std::string& frame_id) {
  return frame_id.empty() || frame_id == kWorldFrame || frame_id == "/world";
}

bool pathExists(const std::string& path) {
  struct stat info;
  return ::stat(path.c_str(), &info) == 0;
}

bool isDirectory(const std::string& path) {
  struct stat info;
  return ::stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

std::string trimTrailingSlashes(std::string path) {
  while (path.size() > 1 && path.back() == '/') path.pop_back();
  return path;
}

struct Point2D {
  double x = 0.0;
  double y = 0.0;
};

struct Primitive {
  enum class Type { WALL, PILLAR };

  Type type = Type::PILLAR;
  Point2D first;
  Point2D second;
  double width = 0.0;
  double base_z = 0.0;
  double height = 0.0;
};

struct Command {
  enum class Type { ADD, CLEAR };

  Type type = Type::ADD;
  std::vector<Primitive> cleared_primitives;
};

struct PointKey {
  long long x = 0;
  long long y = 0;
  long long z = 0;

  bool operator==(const PointKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct PointKeyHash {
  std::size_t operator()(const PointKey& key) const {
    std::size_t seed = std::hash<long long>()(key.x);
    seed ^= std::hash<long long>()(key.y) + 0x9e3779b9 + (seed << 6) +
            (seed >> 2);
    seed ^= std::hash<long long>()(key.z) + 0x9e3779b9 + (seed << 6) +
            (seed >> 2);
    return seed;
  }
};

}  // namespace

class SceneEditor {
 public:
  SceneEditor() : private_node_("~") {
    private_node_.param("resolution", resolution_, 0.1);
    private_node_.param("wall_thickness", wall_thickness_, 0.3);
    private_node_.param("pillar_width", pillar_width_, 0.6);
    private_node_.param("base_z", base_z_, -1.0);
    private_node_.param("height", height_, 4.0);
    private_node_.param<std::string>("output_dir", output_dir_, std::string());

    if (!isFinite(resolution_) || resolution_ < 0.001 || resolution_ > 10.0) {
      throw std::runtime_error("~resolution must be in the range [0.001, 10.0]");
    }

    cloud_publisher_ =
        node_.advertise<sensor_msgs::PointCloud2>("/mock_map", 1, true);
    marker_publisher_ = node_.advertise<visualization_msgs::MarkerArray>(
        "/scene_editor/markers", 1, true);

    wall_subscriber_ = node_.subscribe(
        "/move_base_simple/goal", 10, &SceneEditor::wallPointCallback, this);
    pillar_subscriber_ = node_.subscribe(
        "/clicked_point", 10, &SceneEditor::pillarCallback, this);

    undo_service_ = node_.advertiseService(
        "/scene_editor/undo", &SceneEditor::undoCallback, this);
    clear_service_ = node_.advertiseService(
        "/scene_editor/clear", &SceneEditor::clearCallback, this);
    cancel_wall_service_ = node_.advertiseService(
        "/scene_editor/cancel_wall", &SceneEditor::cancelWallCallback, this);
    save_service_ = node_.advertiseService(
        "/scene_editor/save", &SceneEditor::saveCallback, this);

    rebuildAndPublish();
    publishMarkers();
    ROS_INFO("Scene editor ready: use two 2D Nav Goal clicks for a wall and "
             "Publish Point for a square pillar.");
  }

 private:
  double snapToGrid(double value) const {
    return std::round(value / resolution_) * resolution_;
  }

  bool readRuntimeDimensions(double* wall_thickness, double* pillar_width,
                             double* base_z, double* height,
                             std::string* error) {
    double next_wall_thickness = wall_thickness_;
    double next_pillar_width = pillar_width_;
    double next_base_z = base_z_;
    double next_height = height_;

    private_node_.getParam("wall_thickness", next_wall_thickness);
    private_node_.getParam("pillar_width", next_pillar_width);
    private_node_.getParam("base_z", next_base_z);
    private_node_.getParam("height", next_height);

    if (!isFinite(next_wall_thickness) || next_wall_thickness <= 0.0) {
      *error = "~wall_thickness must be finite and greater than zero";
      return false;
    }
    if (!isFinite(next_pillar_width) || next_pillar_width <= 0.0) {
      *error = "~pillar_width must be finite and greater than zero";
      return false;
    }
    if (!isFinite(next_base_z)) {
      *error = "~base_z must be finite";
      return false;
    }
    if (!isFinite(next_height) || next_height <= 0.0) {
      *error = "~height must be finite and greater than zero";
      return false;
    }
    if (std::abs(next_base_z) > kMaximumCoordinate ||
        std::abs(next_base_z + next_height) > kMaximumCoordinate) {
      *error = "vertical scene coordinates exceed the supported range";
      return false;
    }

    wall_thickness_ = next_wall_thickness;
    pillar_width_ = next_pillar_width;
    base_z_ = next_base_z;
    height_ = next_height;

    *wall_thickness = wall_thickness_;
    *pillar_width = pillar_width_;
    *base_z = base_z_;
    *height = height_;
    return true;
  }

  void wallPointCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    if (!isAcceptedFrame(message->header.frame_id)) {
      ROS_WARN("Ignoring wall point in frame '%s'; RViz fixed frame must be world.",
               message->header.frame_id.c_str());
      return;
    }
    if (!isFinite(message->pose.position.x) ||
        !isFinite(message->pose.position.y) ||
        std::abs(message->pose.position.x) > kMaximumCoordinate ||
        std::abs(message->pose.position.y) > kMaximumCoordinate) {
      ROS_WARN("Ignoring wall point outside the supported coordinate range.");
      return;
    }

    Point2D point{snapToGrid(message->pose.position.x),
                  snapToGrid(message->pose.position.y)};
    if (!has_pending_wall_point_) {
      pending_wall_point_ = point;
      has_pending_wall_point_ = true;
      publishMarkers();
      ROS_INFO("Wall start set to (%.3f, %.3f); select the second endpoint.",
               point.x, point.y);
      return;
    }

    const double dx = point.x - pending_wall_point_.x;
    const double dy = point.y - pending_wall_point_.y;
    if (std::hypot(dx, dy) < resolution_ * 0.5) {
      ROS_WARN("Wall endpoints are too close; the first endpoint is unchanged.");
      return;
    }

    double wall_thickness = 0.0;
    double pillar_width = 0.0;
    double base_z = 0.0;
    double height = 0.0;
    std::string error;
    if (!readRuntimeDimensions(&wall_thickness, &pillar_width, &base_z, &height,
                               &error)) {
      ROS_ERROR("Cannot create wall: %s", error.c_str());
      return;
    }

    Primitive primitive;
    primitive.type = Primitive::Type::WALL;
    primitive.first = pending_wall_point_;
    primitive.second = point;
    primitive.width = wall_thickness;
    primitive.base_z = base_z;
    primitive.height = height;
    if (!addPrimitive(std::move(primitive), &error)) {
      ROS_ERROR("Cannot create wall: %s", error.c_str());
      return;
    }

    has_pending_wall_point_ = false;
    publishMarkers();
    ROS_INFO("Wall added. Scene contains %zu object(s).", primitives_.size());
  }

  void pillarCallback(const geometry_msgs::PointStamped::ConstPtr& message) {
    if (!isAcceptedFrame(message->header.frame_id)) {
      ROS_WARN("Ignoring pillar point in frame '%s'; RViz fixed frame must be world.",
               message->header.frame_id.c_str());
      return;
    }
    if (!isFinite(message->point.x) || !isFinite(message->point.y) ||
        std::abs(message->point.x) > kMaximumCoordinate ||
        std::abs(message->point.y) > kMaximumCoordinate) {
      ROS_WARN("Ignoring pillar point outside the supported coordinate range.");
      return;
    }

    double wall_thickness = 0.0;
    double pillar_width = 0.0;
    double base_z = 0.0;
    double height = 0.0;
    std::string error;
    if (!readRuntimeDimensions(&wall_thickness, &pillar_width, &base_z, &height,
                               &error)) {
      ROS_ERROR("Cannot create pillar: %s", error.c_str());
      return;
    }

    Primitive primitive;
    primitive.type = Primitive::Type::PILLAR;
    primitive.first = Point2D{snapToGrid(message->point.x),
                              snapToGrid(message->point.y)};
    primitive.second = primitive.first;
    primitive.width = pillar_width;
    primitive.base_z = base_z;
    primitive.height = height;
    if (!addPrimitive(std::move(primitive), &error)) {
      ROS_ERROR("Cannot create pillar: %s", error.c_str());
      return;
    }
    ROS_INFO("Pillar added. Scene contains %zu object(s).", primitives_.size());
  }

  long double primitiveSampleEstimate(const Primitive& primitive) const {
    const long double width_count = std::max(
        1.0L, std::ceil(static_cast<long double>(primitive.width) / resolution_));
    const long double height_count = std::max(
        1.0L, std::ceil(static_cast<long double>(primitive.height) / resolution_));
    if (primitive.type == Primitive::Type::PILLAR) {
      return width_count * width_count * height_count;
    }
    const long double length =
        std::hypot(primitive.second.x - primitive.first.x,
                   primitive.second.y - primitive.first.y);
    const long double length_count =
        std::max(1.0L, std::ceil(length / resolution_)) + 1.0L;
    return length_count * width_count * height_count;
  }

  bool addPrimitive(Primitive primitive, std::string* error) {
    long double total_samples = primitiveSampleEstimate(primitive);
    for (const Primitive& existing : primitives_) {
      total_samples += primitiveSampleEstimate(existing);
      if (total_samples > kMaximumSceneSamples) break;
    }
    if (!std::isfinite(total_samples) ||
        total_samples > kMaximumSceneSamples) {
      *error = "scene would exceed the 20,000,000-point safety limit";
      return false;
    }

    primitives_.push_back(std::move(primitive));
    Command command;
    command.type = Command::Type::ADD;
    history_.push_back(std::move(command));
    rebuildAndPublish();
    return true;
  }

  bool undoCallback(std_srvs::Trigger::Request&,
                    std_srvs::Trigger::Response& response) {
    if (history_.empty()) {
      response.success = true;
      response.message = "Nothing to undo.";
      return true;
    }

    Command command = std::move(history_.back());
    history_.pop_back();
    if (command.type == Command::Type::ADD) {
      if (primitives_.empty()) {
        response.success = false;
        response.message = "Internal history mismatch.";
        return true;
      }
      primitives_.pop_back();
      response.message = "Removed the most recently created object.";
    } else {
      primitives_ = std::move(command.cleared_primitives);
      response.message = "Restored the scene cleared by the last clear command.";
    }

    response.success = true;
    rebuildAndPublish();
    return true;
  }

  bool clearCallback(std_srvs::Trigger::Request&,
                     std_srvs::Trigger::Response& response) {
    has_pending_wall_point_ = false;
    publishMarkers();
    if (primitives_.empty()) {
      response.success = true;
      response.message = "Scene is already empty.";
      return true;
    }

    Command command;
    command.type = Command::Type::CLEAR;
    command.cleared_primitives = std::move(primitives_);
    history_.push_back(std::move(command));
    primitives_.clear();
    rebuildAndPublish();

    response.success = true;
    response.message = "Scene cleared; one undo will restore it.";
    return true;
  }

  bool cancelWallCallback(std_srvs::Trigger::Request&,
                          std_srvs::Trigger::Response& response) {
    if (!has_pending_wall_point_) {
      response.success = true;
      response.message = "No wall endpoint is pending.";
      return true;
    }

    has_pending_wall_point_ = false;
    publishMarkers();
    response.success = true;
    response.message = "Pending wall endpoint cancelled.";
    return true;
  }

  bool saveCallback(map_generator::SaveScene::Request& request,
                    map_generator::SaveScene::Response& response) {
    response.success = false;
    response.path.clear();

    if (cloud_.empty()) {
      response.message = "Cannot save an empty scene.";
      return true;
    }

    std::string base_name = request.name;
    const std::string suffix = ".pcd";
    if (base_name.size() > suffix.size() &&
        base_name.compare(base_name.size() - suffix.size(), suffix.size(), suffix) ==
            0) {
      base_name.resize(base_name.size() - suffix.size());
    }
    static const std::regex valid_name("^[A-Za-z0-9_-]+$");
    if (!std::regex_match(base_name, valid_name)) {
      response.message =
          "Scene name must contain only letters, numbers, '_' and '-'.";
      return true;
    }

    private_node_.getParam("output_dir", output_dir_);
    output_dir_ = trimTrailingSlashes(output_dir_);
    if (output_dir_.empty() || output_dir_.front() != '/') {
      response.message = "~output_dir must be an absolute directory path.";
      return true;
    }
    if (!isDirectory(output_dir_)) {
      response.message = "Output directory does not exist or is not a directory: " +
                         output_dir_;
      return true;
    }

    const std::string target_path = output_dir_ + "/" + base_name + suffix;
    if (pathExists(target_path) && !request.overwrite) {
      response.message = "Target already exists; set overwrite=true to replace it.";
      response.path = target_path;
      return true;
    }

    std::string temporary_template =
        output_dir_ + "/." + base_name + ".tmp-XXXXXX" + suffix;
    std::vector<char> temporary_buffer(temporary_template.begin(),
                                       temporary_template.end());
    temporary_buffer.push_back('\0');
    const int temporary_fd = ::mkstemps(temporary_buffer.data(), suffix.size());
    if (temporary_fd < 0) {
      response.message =
          "Failed to create temporary file: " + std::string(std::strerror(errno));
      return true;
    }
    const std::string temporary_path(temporary_buffer.data());
    if (::close(temporary_fd) != 0) {
      const std::string reason = std::strerror(errno);
      std::remove(temporary_path.c_str());
      response.message = "Failed to close temporary file: " + reason;
      return true;
    }

    int save_status = -1;
    try {
      save_status = pcl::io::savePCDFileASCII(temporary_path, cloud_);
    } catch (const std::exception& error) {
      std::remove(temporary_path.c_str());
      response.message = "PCL failed to write temporary PCD file: " +
                         std::string(error.what());
      return true;
    } catch (...) {
      std::remove(temporary_path.c_str());
      response.message = "PCL failed to write temporary PCD file.";
      return true;
    }
    if (save_status != 0) {
      std::remove(temporary_path.c_str());
      response.message = "PCL failed to write temporary PCD file.";
      return true;
    }
    ::chmod(temporary_path.c_str(), S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP |
                                        S_IROTH);

    if (request.overwrite) {
      if (std::rename(temporary_path.c_str(), target_path.c_str()) != 0) {
        const std::string reason = std::strerror(errno);
        std::remove(temporary_path.c_str());
        response.message = "Failed to finalize PCD file: " + reason;
        return true;
      }
    } else {
      // A hard link publishes the same-directory temporary file atomically and
      // fails with EEXIST instead of replacing a concurrently created target.
      if (::link(temporary_path.c_str(), target_path.c_str()) != 0) {
        const int link_error = errno;
        std::remove(temporary_path.c_str());
        response.path = target_path;
        if (link_error == EEXIST) {
          response.message =
              "Target appeared while saving; no file was replaced.";
        } else {
          response.message =
              "Failed to finalize PCD file: " +
              std::string(std::strerror(link_error));
        }
        return true;
      }
      if (std::remove(temporary_path.c_str()) != 0) {
        ROS_WARN("Saved scene, but could not remove temporary link %s: %s",
                 temporary_path.c_str(), std::strerror(errno));
      }
    }

    response.success = true;
    response.path = target_path;
    response.message = "Scene saved as ASCII PCD.";
    ROS_INFO("Saved scene with %zu points to %s", cloud_.size(),
             target_path.c_str());
    return true;
  }

  int sampleCount(double dimension) const {
    return std::max(1, static_cast<int>(std::ceil(dimension / resolution_)));
  }

  void appendPoint(double x, double y, double z,
                   std::unordered_set<PointKey, PointKeyHash>* seen) {
    // Quantize only the key (to one micrometre) so numerically identical overlap
    // is removed without moving the stored geometry.
    constexpr double kKeyScale = 1000000.0;
    const PointKey key{static_cast<long long>(std::llround(x * kKeyScale)),
                       static_cast<long long>(std::llround(y * kKeyScale)),
                       static_cast<long long>(std::llround(z * kKeyScale))};
    if (!seen->insert(key).second) return;
    cloud_.push_back(pcl::PointXYZ(static_cast<float>(x), static_cast<float>(y),
                                   static_cast<float>(z)));
  }

  void appendPillar(const Primitive& primitive,
                    std::unordered_set<PointKey, PointKeyHash>* seen) {
    const int width_count = sampleCount(primitive.width);
    const int height_count = sampleCount(primitive.height);
    const double width_center = 0.5 * static_cast<double>(width_count - 1);

    for (int x_index = 0; x_index < width_count; ++x_index) {
      const double x = primitive.first.x +
                       (static_cast<double>(x_index) - width_center) * resolution_;
      for (int y_index = 0; y_index < width_count; ++y_index) {
        const double y = primitive.first.y +
                         (static_cast<double>(y_index) - width_center) * resolution_;
        for (int z_index = 0; z_index < height_count; ++z_index) {
          const double z = primitive.base_z +
                           (static_cast<double>(z_index) + 0.5) * resolution_;
          appendPoint(x, y, z, seen);
        }
      }
    }
  }

  void appendWall(const Primitive& primitive,
                  std::unordered_set<PointKey, PointKeyHash>* seen) {
    const double dx = primitive.second.x - primitive.first.x;
    const double dy = primitive.second.y - primitive.first.y;
    const double length = std::hypot(dx, dy);
    if (length <= 0.0) return;

    const double tangent_x = dx / length;
    const double tangent_y = dy / length;
    const double normal_x = -tangent_y;
    const double normal_y = tangent_x;
    const int length_intervals = sampleCount(length);
    const int width_count = sampleCount(primitive.width);
    const int height_count = sampleCount(primitive.height);
    const double width_center = 0.5 * static_cast<double>(width_count - 1);

    for (int length_index = 0; length_index <= length_intervals;
         ++length_index) {
      const double along = length * static_cast<double>(length_index) /
                           static_cast<double>(length_intervals);
      const double center_x = primitive.first.x + tangent_x * along;
      const double center_y = primitive.first.y + tangent_y * along;
      for (int width_index = 0; width_index < width_count; ++width_index) {
        const double offset =
            (static_cast<double>(width_index) - width_center) * resolution_;
        const double x = center_x + normal_x * offset;
        const double y = center_y + normal_y * offset;
        for (int z_index = 0; z_index < height_count; ++z_index) {
          const double z = primitive.base_z +
                           (static_cast<double>(z_index) + 0.5) * resolution_;
          appendPoint(x, y, z, seen);
        }
      }
    }
  }

  void rebuildAndPublish() {
    cloud_.clear();
    std::unordered_set<PointKey, PointKeyHash> seen;
    for (const Primitive& primitive : primitives_) {
      if (primitive.type == Primitive::Type::WALL) {
        appendWall(primitive, &seen);
      } else {
        appendPillar(primitive, &seen);
      }
    }

    cloud_.width = static_cast<std::uint32_t>(cloud_.size());
    cloud_.height = 1;
    cloud_.is_dense = true;

    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(cloud_, message);
    message.header.frame_id = kWorldFrame;
    message.header.stamp = ros::Time::now();
    cloud_publisher_.publish(message);
  }

  void publishMarkers() {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    if (has_pending_wall_point_) {
      visualization_msgs::Marker point_marker;
      point_marker.header.frame_id = kWorldFrame;
      point_marker.header.stamp = ros::Time::now();
      point_marker.ns = "scene_editor";
      point_marker.id = 1;
      point_marker.type = visualization_msgs::Marker::SPHERE;
      point_marker.action = visualization_msgs::Marker::ADD;
      point_marker.pose.position.x = pending_wall_point_.x;
      point_marker.pose.position.y = pending_wall_point_.y;
      point_marker.pose.position.z = 0.05;
      point_marker.pose.orientation.w = 1.0;
      point_marker.scale.x = 0.25;
      point_marker.scale.y = 0.25;
      point_marker.scale.z = 0.25;
      point_marker.color.r = 1.0;
      point_marker.color.g = 0.75;
      point_marker.color.b = 0.0;
      point_marker.color.a = 1.0;
      markers.markers.push_back(point_marker);

      visualization_msgs::Marker text_marker = point_marker;
      text_marker.id = 2;
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.pose.position.z = 0.45;
      text_marker.scale.x = 0.0;
      text_marker.scale.y = 0.0;
      text_marker.scale.z = 0.28;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.text = "Select the second wall endpoint";
      markers.markers.push_back(text_marker);
    }

    marker_publisher_.publish(markers);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher cloud_publisher_;
  ros::Publisher marker_publisher_;
  ros::Subscriber wall_subscriber_;
  ros::Subscriber pillar_subscriber_;
  ros::ServiceServer undo_service_;
  ros::ServiceServer clear_service_;
  ros::ServiceServer cancel_wall_service_;
  ros::ServiceServer save_service_;

  double resolution_ = 0.1;
  double wall_thickness_ = 0.3;
  double pillar_width_ = 0.6;
  double base_z_ = -1.0;
  double height_ = 4.0;
  std::string output_dir_;

  bool has_pending_wall_point_ = false;
  Point2D pending_wall_point_;
  std::vector<Primitive> primitives_;
  std::vector<Command> history_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scene_editor");
  try {
    SceneEditor editor;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Failed to start scene editor: %s", error.what());
    return 1;
  }
  return 0;
}
