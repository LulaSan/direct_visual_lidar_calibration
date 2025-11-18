#include <vlcal/common/visual_lidar_data.hpp>

#include <opencv2/opencv.hpp>
#include <vlcal/common/console_colors.hpp>

#include <glk/io/ply_io.hpp>

namespace vlcal {

VisualLiDARData::VisualLiDARData(const std::string& data_path, const std::string& bag_name) {
  std::cout << "loading " << data_path + "/" + bag_name + ".(png|ply)" << std::endl;

  const std::string image_path = data_path + "/" + bag_name + "/image.png";
  image = cv::imread(image_path, cv::IMREAD_COLOR); 
  if (!image.data) {
    std::cerr << vlcal::console::bold_red << "warning: failed to load " << data_path + "/" + bag_name + ".png" << vlcal::console::reset << std::endl;
  }
  const std::string alt_image_path = data_path + "/" + bag_name + ".png";
    std::cout << "Trying alternative path: " << alt_image_path << std::endl;
    image = cv::imread(alt_image_path, cv::IMREAD_COLOR);
    
    if (!image.data) {
      std::cerr << vlcal::console::bold_red << "error: failed to load image from both paths" << vlcal::console::reset << std::endl;
      abort();
    }
  auto ply = glk::load_ply(data_path + "/" + bag_name + ".ply");
  if (!ply) {
    std::cerr << vlcal::console::bold_red << "warning: failed to load " << data_path + "/" + bag_name + ".ply" << vlcal::console::reset << std::endl;
    abort();
  }

  points = std::make_shared<FrameCPU>(ply->vertices);
  points->add_intensities(ply->intensities);
}

VisualLiDARData::~VisualLiDARData() {}

}