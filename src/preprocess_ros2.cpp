#include <vlcal/preprocess/preprocess.hpp>

#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#ifdef CV_BRIDGE_INCLUDE_H
#include <cv_bridge/cv_bridge.h>
#elif CV_BRIDGE_INCLUDE_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#error "File extension of cv_bridge is unknown!!"
#endif

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#define VLCAL_ROS2
#include <vlcal/common/ros_cloud_converter.hpp>

namespace vlcal {

class PointCloudReaderROS2 : public PointCloudReader {
public:
  PointCloudReaderROS2(const std::string& bag_filename, const std::string& points_topic, const std::string& intensity_channel) : intensity_channel(intensity_channel) {
    std::cout << "Initializing PointCloudReaderROS2 for bag: " << bag_filename << ", topic: " << points_topic << std::endl;
    
    try {
      reader.open(bag_filename);
      std::cout << "Successfully opened bag for point cloud reading" << std::endl;

      rosbag2_storage::StorageFilter filter;
      filter.topics.push_back(points_topic);
      reader.set_filter(filter);
      std::cout << "Set filter for topic: " << points_topic << std::endl;
      
      // Check if there are any messages with this topic
      if (reader.has_next()) {
        std::cout << "Bag has messages available" << std::endl;
      } else {
        std::cout << "Warning: No messages available in bag for topic: " << points_topic << std::endl;
      }
    } catch (const std::exception& e) {
      std::cerr << "Error initializing PointCloudReaderROS2: " << e.what() << std::endl;
    }
  }

  virtual RawPoints::Ptr read_next() override {
    if (!reader.has_next()) {
      std::cout << "No more messages in bag" << std::endl;
      return nullptr;
    }

    try {
      rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pointcloud_serialization;

      const auto msg = reader.read_next();
      std::cout << "Read message with topic: " << msg->topic_name << std::endl;
      
      const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

      auto points_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pointcloud_serialization.deserialize_message(&serialized_msg, points_msg.get());

      std::cout << "Deserialized point cloud with " << points_msg->width * points_msg->height << " points" << std::endl;

      auto raw_points = extract_raw_points(points_msg, intensity_channel);
      if (raw_points) {
        std::cout << "Extracted " << raw_points->size() << " raw points" << std::endl;
      } else {
        std::cout << "Failed to extract raw points" << std::endl;
      }
      
      return raw_points;
    } catch (const std::exception& e) {
      std::cerr << "Error reading point cloud: " << e.what() << std::endl;
      return nullptr;
    }
  }

private:
  const std::string intensity_channel;
  rosbag2_cpp::Reader reader;
};

class PreprocessROS2 : public Preprocess {
protected:
  template <typename T>
  std::shared_ptr<T> get_first_message(const std::string& bag_filename, const std::string& topic) const {
    rosbag2_cpp::Reader reader;
    reader.open(bag_filename);

    rosbag2_storage::StorageFilter filter;
    filter.topics.emplace_back(topic);
    reader.set_filter(filter);

    if (!reader.has_next()) {
      std::cerr << "error: bag does not contain topic " << std::endl;
      std::cerr << "     : bag_filename=" << bag_filename << std::endl;
      std::cerr << "     : topic=" << topic << std::endl;
      abort();
    }

    rclcpp::Serialization<T> serialization;
    const auto msg = reader.read_next();
    const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

    auto deserialized = std::make_shared<T>();
    serialization.deserialize_message(&serialized_msg, deserialized.get());
    return deserialized;
  }

  virtual bool valid_bag(const std::string& bag_filename) override {
    // Check if it's a directory (traditional rosbag2 format) or .mcap file
    if (std::filesystem::is_directory(bag_filename)) {
      // Traditional rosbag2 format (directory with metadata.yaml)
      std::string metadata_path = bag_filename + "/metadata.yaml";
      if (!std::filesystem::exists(metadata_path)) {
        return false;
      }
    } else if (bag_filename.size() >= 5 && bag_filename.substr(bag_filename.size() - 5) == ".mcap") {
      // MCAP file format
      if (!std::filesystem::exists(bag_filename)) {
        return false;
      }
    } else {
      // Unknown format
      return false;
    }

    try {
      rosbag2_cpp::Reader reader;
      reader.open(bag_filename);
    } catch (std::exception& e) {
      return false;
    }

    return true;
  }

  virtual std::vector<std::pair<std::string, std::string>> get_topics_and_types(const std::string& bag_filename) override {
    rosbag2_cpp::Reader reader;
    reader.open(bag_filename);
    std::vector<std::pair<std::string, std::string>> topics_and_types;
    for (const auto& topic_metadata : reader.get_all_topics_and_types()) {
      topics_and_types.emplace_back(topic_metadata.name, topic_metadata.type);
    }

    return topics_and_types;
  }

  virtual std::vector<std::string> get_point_fields(const std::string& bag_filename, const std::string& points_topic) override {
    const auto msg = get_first_message<sensor_msgs::msg::PointCloud2>(bag_filename, points_topic);
    std::vector<std::string> fields(msg->fields.size());
    std::transform(msg->fields.begin(), msg->fields.end(), fields.begin(), [](const auto& field) { return field.name; });
    return fields;
  }

  virtual cv::Size get_image_size(const std::string& bag_filename, const std::string& image_topic) override {
    if (image_topic.find("compressed") == std::string::npos) {
      const auto image_msg = get_first_message<sensor_msgs::msg::Image>(bag_filename, image_topic);
      return cv::Size(image_msg->width, image_msg->height);
    }

    const auto image_msg = get_first_message<sensor_msgs::msg::CompressedImage>(bag_filename, image_topic);
    return cv_bridge::toCvCopy(*image_msg, "bgr8")->image.size();
  }

  virtual std::tuple<std::string, std::vector<double>, std::vector<double>> get_camera_info(const std::string& bag_filename, const std::string& camera_info_topic) override {
    const auto camera_info_msg = get_first_message<sensor_msgs::msg::CameraInfo>(bag_filename, camera_info_topic);
    std::vector<double> intrinsic(4);
    intrinsic[0] = camera_info_msg->k[0];
    intrinsic[1] = camera_info_msg->k[4];
    intrinsic[2] = camera_info_msg->k[2];
    intrinsic[3] = camera_info_msg->k[5];
    std::vector<double> distortion_coeffs = camera_info_msg->d;

    return {camera_info_msg->distortion_model, intrinsic, distortion_coeffs};
  }

  virtual cv::Mat get_image(const std::string& bag_filename, const std::string& image_topic) override {
    std::cout << "Getting image from bag: " << bag_filename << ", topic: " << image_topic << std::endl;
    
    try {
      if (image_topic.find("compressed") == std::string::npos) {
        const auto image_msg = get_first_message<sensor_msgs::msg::Image>(bag_filename, image_topic);
        std::cout << "Retrieved uncompressed image: " << image_msg->width << "x" << image_msg->height << std::endl;
        // return cv_bridge::toCvCopy(*image_msg, "mono8")->image;
        return cv_bridge::toCvCopy(*image_msg, "bgr8")->image;

      }

      const auto image_msg = get_first_message<sensor_msgs::msg::CompressedImage>(bag_filename, image_topic);
      std::cout << "Retrieved compressed image" << std::endl;
      return cv_bridge::toCvCopy(*image_msg, "bgr8")->image;
    } catch (const std::exception& e) {
      std::cerr << "Error getting image: " << e.what() << std::endl;
      return cv::Mat();
    }
  }

  virtual std::shared_ptr<PointCloudReader> get_point_cloud_reader(const std::string& bag_filename, const std::string& points_topic, const std::string& intensity_channel)
    override {
    std::cout << "About to create PointCloudReaderROS2..." << std::endl;
    try {
      auto reader = std::make_shared<PointCloudReaderROS2>(bag_filename, points_topic, intensity_channel);
      std::cout << "Successfully created PointCloudReaderROS2" << std::endl;
      return reader;
    } catch (const std::exception& e) {
      std::cerr << "Exception creating PointCloudReaderROS2: " << e.what() << std::endl;
      throw;
    }
  }
};

}  // namespace vlcal

int main(int argc, char** argv) {
  vlcal::PreprocessROS2 preprocess;
  preprocess.run(argc, argv);

  return 0;
}
