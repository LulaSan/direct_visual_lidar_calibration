#include <vlcal/common/points_color_updater.hpp>
#include <vlcal/common/estimate_fov.hpp>

#include <glk/primitives/icosahedron.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace vlcal {

PointsColorUpdater::PointsColorUpdater(const camera::GenericCameraBase::ConstPtr& proj, const cv::Mat& image)
: proj(proj),
  min_nz(std::cos(estimate_camera_fov(proj, {image.cols, image.rows}) + 0.5 * M_PI / 180.0)),
  image(image) {
  glk::Icosahedron icosahedron;
  for (int i = 0; i < 6; i++) {
    icosahedron.subdivide();
  }
  icosahedron.spherize();

  points = std::make_shared<FrameCPU>(icosahedron.vertices);
  intensity_colors.resize(points->size(), Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f));

  cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points->points, points->size());
}

PointsColorUpdater::PointsColorUpdater(const camera::GenericCameraBase::ConstPtr& proj, const cv::Mat& image, const FrameCPU::ConstPtr& points)
: proj(proj),
  min_nz(std::cos(estimate_camera_fov(proj, {image.cols, image.rows}) + 0.5 * M_PI / 180.0)),
  image(image),
  points(points) {
  cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points->points, points->size());
  intensity_colors.resize(points->size());
  for (int i = 0; i < points->size(); i++) {
    intensity_colors[i] = glk::colormapf(glk::COLORMAP::TURBO, points->intensities[i]);
  }
}

// void PointsColorUpdater::update(const Eigen::Isometry3d& T_camera_liar, const double blend_weight) {
//   std::shared_ptr<std::vector<Eigen::Vector4f>> colors(new std::vector<Eigen::Vector4f>(points->size(), Eigen::Vector4f::Zero()));

//   for (int i = 0; i < points->size(); i++) {
//     const Eigen::Vector4d pt_camera = T_camera_liar * points->points[i];

//     if (pt_camera.head<3>().normalized().z() < min_nz) {
//       // Out of FoV
//       continue;
//     }

//     const Eigen::Vector2i pt_2d = proj->project(pt_camera.head<3>()).cast<int>();
//     if ((pt_2d.array() < Eigen::Array2i::Zero()).any() || (pt_2d.array() >= Eigen::Array2i(image.cols, image.rows)).any()) {
//       // Out of Image
//       continue;
//     }

//     const unsigned char pix = image.at<std::uint8_t>(pt_2d.y(), pt_2d.x());
//     const Eigen::Vector4f color(pix / 255.0f, pix / 255.0f, pix / 255.0f, 1.0f);

//     colors->at(i) = color * blend_weight + intensity_colors[i] * (1.0 - blend_weight);
//   }

//   guik::LightViewer::instance()->invoke([cloud_buffer = cloud_buffer, colors = colors] { cloud_buffer->add_color(*colors); });
// }
void PointsColorUpdater::update(const Eigen::Isometry3d& T_camera_lidar, const double blend_weight) {
  // Clamp blend weight to avoid fully washing out either source
  const double w = std::clamp(blend_weight, 0.0, 1.0);

  std::shared_ptr<std::vector<Eigen::Vector4f>> colors(new std::vector<Eigen::Vector4f>(points->size(), Eigen::Vector4f::Zero()));

  const bool is_gray = (image.type() == CV_8UC1);
  const bool is_bgr  = (image.type() == CV_8UC3);

  for (int i = 0; i < points->size(); i++) {
    const Eigen::Vector4d pt_camera = T_camera_lidar * points->points[i];

    if (pt_camera.head<3>().normalized().z() < min_nz) {
      continue; // out of FoV
    }

    const Eigen::Vector2i pt_2d = proj->project(pt_camera.head<3>()).cast<int>();
    if ((pt_2d.array() < Eigen::Array2i::Zero()).any() || (pt_2d.array() >= Eigen::Array2i(image.cols, image.rows)).any()) {
      continue; // out of image
    }

    Eigen::Vector4f img_color(0,0,0,1);
    if (is_gray) {
      const uint8_t pix = image.at<uint8_t>(pt_2d.y(), pt_2d.x());
      const float v = pix / 255.0f;
      img_color.head<3>() = Eigen::Vector3f(v, v, v);
    } else if (is_bgr) {
      const cv::Vec3b bgr = image.at<cv::Vec3b>(pt_2d.y(), pt_2d.x());
      // Convert BGR to RGB normalized
      img_color[0] = bgr[2] / 255.0f; // R
      img_color[1] = bgr[1] / 255.0f; // G
      img_color[2] = bgr[0] / 255.0f; // B
    } else {
      // Unsupported format; use intensity color only
      colors->at(i) = intensity_colors[i];
      continue;
    }

    // Intensity color already stored in intensity_colors[i] (RGBA)
    const Eigen::Vector4f intensity_color = intensity_colors[i];

    // Linear blend between image color and intensity color
    colors->at(i) = img_color * static_cast<float>(w) + intensity_color * (1.0f - static_cast<float>(w));
    colors->at(i)[3] = 1.0f; // Ensure alpha is 1
  }

  guik::LightViewer::instance()->invoke([cloud_buffer = cloud_buffer, colors = colors] {
    cloud_buffer->add_color(*colors);
  });
}
}  // namespace vlcal
