#include <fstream>
#include <iostream>
#include <boost/program_options.hpp>

#include <nlohmann/json.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <camera/create_camera.hpp>
#include <vlcal/common/console_colors.hpp>
#include <vlcal/common/visual_lidar_visualizer.hpp>
#include <guik/hovered_drawings.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/texture_opencv.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>
namespace vlcal {

class Viewer {
public:
  Viewer(const std::string& data_path) : data_path(data_path) {
    std::ifstream ifs(data_path + "/calib.json");
    if (!ifs) {
      std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path << "/calib.json" << vlcal::console::reset << std::endl;
      abort();
    }

    ifs >> config;

    auto viewer = guik::LightViewer::instance();
    viewer->invoke([] {
      ImGui::SetNextWindowPos({55, 60}, ImGuiCond_Once);
      ImGui::Begin("visualizer");
      ImGui::End();
      ImGui::SetNextWindowPos({55, 150}, ImGuiCond_Once);
      ImGui::Begin("data selection");
      ImGui::End();
      ImGui::SetNextWindowPos({55, 300}, ImGuiCond_Once);
      ImGui::Begin("texts");
      ImGui::End();
      ImGui::SetNextWindowPos({1260, 60}, ImGuiCond_Once);
      ImGui::Begin("images");
      ImGui::End();
    });

    cv::namedWindow("image");
    viewer->register_ui_callback("callback", [this] { ui_callback(); });

    const auto tum2pose = [](const std::vector<double>& values) {
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() << values[0], values[1], values[2];
      pose.linear() = Eigen::Quaterniond(values[6], values[3], values[4], values[5]).toRotationMatrix();
      return pose;
    };

    if (config.count("results") && config["results"].count("init_T_lidar_camera_auto")) {
      viewer->append_text("Automatic initial guess result found");
      std::cout << "Automatic initial guess result found" << std::endl;
      const std::vector<double> values = config["results"]["init_T_lidar_camera_auto"];
      T_labels.emplace_back("INIT_GUESS (AUTO)");
      T_lidar_camera.emplace_back(tum2pose(values));
      std::cout << "--- T_lidar_camera ---" << std::endl << T_lidar_camera.back().matrix() << std::endl;
    }

    if (config.count("results") && config["results"].count("init_T_lidar_camera")) {
      viewer->append_text("Manual initial guess result found");
      std::cout << "Manual initial guess result found" << std::endl;
      const std::vector<double> values = config["results"]["init_T_lidar_camera"];
      T_labels.emplace_back("INIT_GUESS (MANUAL)");
      T_lidar_camera.emplace_back(tum2pose(values));
      std::cout << "--- T_lidar_camera ---" << std::endl << T_lidar_camera.back().matrix() << std::endl;
    }

    if (config.count("results") && config["results"].count("T_lidar_camera")) {
      viewer->append_text("Calibration result found");
      std::cout << "Calibration result found" << std::endl;
      const std::vector<double> values = config["results"]["T_lidar_camera"];
      T_labels.emplace_back("CALIBRATION_RESULT");
      T_lidar_camera.emplace_back(tum2pose(values));
      std::cout << "--- T_lidar_camera ---" << std::endl << T_lidar_camera.back().matrix() << std::endl;
    }

    if (T_labels.empty()) {
      viewer->append_text("error: no transformation found in calib.json!!");
      std::cerr << vlcal::console::bold_red << "error: no transformation found in calib.json!!" << vlcal::console::reset << std::endl;
      T_labels.emplace_back("NONE");
      T_lidar_camera.emplace_back(Eigen::Isometry3d::Identity());
    }

    selected_transformation = T_labels.size() - 1;

    const std::string camera_model = config["camera"]["camera_model"];
    const std::vector<double> intrinsics = config["camera"]["intrinsics"];
    const std::vector<double> distortion_coeffs = config["camera"]["distortion_coeffs"];
    proj = camera::create_camera(camera_model, intrinsics, distortion_coeffs);

    const std::vector<std::string> bag_names = config["meta"]["bag_names"];
    for (const auto& bag_name : bag_names) {
      dataset.emplace_back(std::make_shared<VisualLiDARData>(data_path, bag_name));
    }

    vis.reset(new VisualLiDARVisualizer(proj, dataset, true));
    vis->set_T_camera_lidar(T_lidar_camera[selected_transformation].inverse());

    point_size = 1.0f;                 // initialize point size
    vis->set_point_scale(point_size);  // apply to visualizer

    viewer->register_ui_callback("callback", [this] { ui_callback(); });
  }

  void ui_callback() {
    ImGui::Begin("visualizer", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    if (ImGui::DragFloat("Point scale", &point_size, 0.1f, 1.0f, 10.0f)) {
      vis->set_point_scale(point_size);
    }
    ImGui::End();
    ImGui::Begin("data selection", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    std::vector<const char*> labels(T_labels.size());
    std::transform(T_labels.begin(), T_labels.end(), labels.begin(), [](const auto& x) { return x.c_str(); });

    if (ImGui::Combo("Transformation", &selected_transformation, labels.data(), labels.size())) {
      vis->set_T_camera_lidar(T_lidar_camera[selected_transformation].inverse());
    }
    ImGui::End();
    ImGui::Begin("Projection Error Check", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    if (picked_pt_3d.has_value()) {
      ImGui::Text("Picked 3D Point:");
      ImGui::Text("  X: %.3f", picked_pt_3d->x());
      ImGui::Text("  Y: %.3f", picked_pt_3d->y());
      ImGui::Text("  Z: %.3f", picked_pt_3d->z());

      if (ImGui::Button("Project to Image")) {
        project_and_display();
      }

      if (has_projection) {
        ImGui::Text("Projected at: (%.1f, %.1f)", projected_point.x(), projected_point.y());
      }
    } else {
      ImGui::Text("Right-click 3D point to select");
    }
    ImGui::End();

    // Update image display continuously
    update_image_display();
  }
  void update_image_display() {
    const double scale = vis->get_image_display_scale();
    const int current_bag = vis->get_selected_bag_id();

    cv::Mat display_image;
    cv::resize(dataset[current_bag]->image, display_image, cv::Size(), scale, scale);

    // Draw projection circle if available
    if (has_projection && picked_pt_3d.has_value()) {
      draw_projection_circle(display_image, scale);
    }

    cv::imshow("image", display_image);
    cv::waitKey(1);
  }
  void project_and_display() {
    if (!picked_pt_3d.has_value()) return;

    // Transform to camera coordinates
    const Eigen::Isometry3d& T = T_lidar_camera[selected_transformation].inverse();
    const Eigen::Vector3d pt_camera = T * picked_pt_3d->head<3>();

    // Check if point is visible
    if (pt_camera.z() <= 0) {
      std::cout << "Point behind camera" << std::endl;
      has_projection = false;
      return;
    }

    // Project to image plane
    projected_point = proj->project(pt_camera);

    // Check bounds
    const cv::Size img_size = dataset[vis->get_selected_bag_id()]->image.size();
    if (projected_point.x() < 0 || projected_point.y() < 0 || projected_point.x() >= img_size.width || projected_point.y() >= img_size.height) {
      std::cout << "Projection outside image bounds" << std::endl;
      has_projection = false;
      return;
    }

    has_projection = true;
    std::cout << "3D point (" << pt_camera.x() << ", " << pt_camera.y() << ", " << pt_camera.z() << ") projects to (" << projected_point.x() << ", " << projected_point.y() << ")"
              << std::endl;
  }

  void draw_projection_circle(cv::Mat& image, double scale) {
    const int radius = 12;
    const int x = static_cast<int>(projected_point.x() * scale);
    const int y = static_cast<int>(projected_point.y() * scale);
    
    if (x >= radius && y >= radius && x < image.cols - radius && y < image.rows - radius) {
        // Draw circle with outline for visibility
        cv::circle(image, cv::Point(x, y), radius, cv::Scalar(0, 0, 0), 3, cv::LINE_AA); // Black outline
        cv::circle(image, cv::Point(x, y), radius, cv::Scalar(0, 255, 0), 2, cv::LINE_AA); // Green circle
        
        // Optional: Add a small center dot
        cv::circle(image, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1); // Filled green dot
    }
}

  void spin() {
    auto viewer = guik::LightViewer::instance();
    

    // Register the point picking callback once
    viewer->register_ui_callback("point_picker", [&] {
      const auto& io = ImGui::GetIO();
      if (!io.WantCaptureMouse && io.MouseClicked[1]) {
        const float depth = viewer->pick_depth({io.MousePos[0], io.MousePos[1]});
        if (depth > -1.0f && depth < 1.0f) {
          const Eigen::Vector3f pt_3d = viewer->unproject({io.MousePos[0], io.MousePos[1]}, depth);
          picked_pt_3d = Eigen::Vector4d(pt_3d.x(), pt_3d.y(), pt_3d.z(), 1.0);
          has_projection = false;
          std::cout << "Selected 3D point: (" << pt_3d.x() << ", " << pt_3d.y() << ", " << pt_3d.z() << ")" << std::endl;
          
          guik::HoveredDrawings hovered;
          hovered.add_cross(pt_3d, IM_COL32(64, 64, 64, 255), 15.0f, 4.0f);
          hovered.add_cross(pt_3d, IM_COL32(0, 255, 0, 255), 15.0f, 3.0f);
          viewer->register_ui_callback("hovered", hovered.create_callback());
        }
      }
    });

    // Use the same pattern as the working initial_guess_manual
    while (vis->spin_once()) {
      cv::waitKey(1);
    }
  }

private:
  cv::Mat current_displayed_image;  // Store current image with projections
  bool has_projection = false;
  Eigen::Vector2d projected_point;
  const std::string data_path;
  nlohmann::json config;
  float point_size;
  camera::GenericCameraBase::ConstPtr proj;
  std::vector<VisualLiDARData::ConstPtr> dataset;
  std::optional<Eigen::Vector4d> picked_pt_3d;
  std::unique_ptr<VisualLiDARVisualizer> vis;

  int selected_transformation;
  std::vector<std::string> T_labels;
  std::vector<Eigen::Isometry3d> T_lidar_camera;
};

}  // namespace vlcal

int main(int argc, char** argv) {
  using namespace boost::program_options;
  options_description description("viewer");

  // clang-format off
  description.add_options()
    ("help", "produce help message")
    ("data_path", value<std::string>(), "directory that contains preprocessed data")
  ;
  // clang-format on

  positional_options_description p;
  p.add("data_path", 1);

  variables_map vm;
  store(command_line_parser(argc, argv).options(description).positional(p).run(), vm);
  notify(vm);

  if (vm.count("help") || !vm.count("data_path")) {
    std::cout << description << std::endl;
    return 0;
  }

  const std::string data_path = vm["data_path"].as<std::string>();
  vlcal::Viewer viewer(data_path);
  viewer.spin();
}