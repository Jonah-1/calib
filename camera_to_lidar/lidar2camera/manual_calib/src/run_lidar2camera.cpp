/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <string>

#include "extrinsic_param.hpp"
#include "intrinsic_param.hpp"
#include "projector_lidar.hpp"

// JSON解析
#include <json/json.h>

using namespace std;

// 3D框结构
struct Box3D {
  Eigen::Vector3d center;
  Eigen::Vector3d x_dir;
  Eigen::Vector3d y_dir;
  Eigen::Vector3d z_dir;
  Eigen::Vector3d size; // [length, width, height]
  std::string name;
  
  // 计算8个顶点
  std::vector<Eigen::Vector3d> getCorners() const {
    std::vector<Eigen::Vector3d> corners;
    double half_x = size[0] / 2.0;
    double half_y = size[1] / 2.0;
    double half_z = size[2] / 2.0;
    
    // 8个顶点的局部坐标
    std::vector<Eigen::Vector3d> local_corners = {
      Eigen::Vector3d(-half_x, -half_y, -half_z),
      Eigen::Vector3d( half_x, -half_y, -half_z),
      Eigen::Vector3d( half_x,  half_y, -half_z),
      Eigen::Vector3d(-half_x,  half_y, -half_z),
      Eigen::Vector3d(-half_x, -half_y,  half_z),
      Eigen::Vector3d( half_x, -half_y,  half_z),
      Eigen::Vector3d( half_x,  half_y,  half_z),
      Eigen::Vector3d(-half_x,  half_y,  half_z)
    };
    
    // 转换到世界坐标系
    for (const auto& local_corner : local_corners) {
      Eigen::Vector3d world_corner = center + 
                                     local_corner[0] * x_dir +
                                     local_corner[1] * y_dir +
                                     local_corner[2] * z_dir;
      corners.push_back(world_corner);
    }
    
    return corners;
  }
};

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049
#define APPLY_COLOR_TO_LIDAR_INTENSITY // to set intensity colored or not

double cali_scale_degree_ = 0.3;
double cali_scale_trans_ = 0.06;
double cali_scale_fxfy_ = 1.005;
static Eigen::Matrix4d calibration_matrix_ = Eigen::Matrix4d::Identity();
static Eigen::Matrix4d orign_calibration_matrix_ = Eigen::Matrix4d::Identity();
static Eigen::Matrix3d intrinsic_matrix_ = Eigen::Matrix3d::Identity();
static Eigen::Matrix3d orign_intrinsic_matrix_ = Eigen::Matrix3d::Identity();
std::vector<float> distortions_;
std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Vector4d>> modification_list_(12);
bool display_mode_ = false;
bool filter_mode_ = false;

bool kbhit() {
  termios term;
  tcgetattr(0, &term);
  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);
  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);
  tcsetattr(0, TCSANOW, &term);
  return byteswaiting > 0;
}

void CalibrationInit(Eigen::Matrix4d json_param) {
  Eigen::Matrix4d init_cali;
  init_cali << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  calibration_matrix_ = json_param;
  orign_calibration_matrix_ = json_param;
  modification_list_.resize(12);
  for (int32_t i = 0; i < 12; i++) {
    std::vector<int> transform_flag(6, 0);
    transform_flag[i / 2] = (i % 2) ? (-1) : 1;
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rot_tmp;
    rot_tmp =
        Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitZ());
    tmp.block(0, 0, 3, 3) = rot_tmp;
    tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
    tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
    tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
    modification_list_[i] = tmp;
  }
  std::cout << "=>Calibration scale Init!\n";
}

void CalibrationScaleChange() {
  Eigen::Matrix4d init_cali;
  init_cali << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  for (int32_t i = 0; i < 12; i++) {
    std::vector<int> transform_flag(6, 0);
    transform_flag[i / 2] = (i % 2) ? (-1) : 1;
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rot_tmp;
    rot_tmp =
        Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitZ());
    tmp.block(0, 0, 3, 3) = rot_tmp;
    tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
    tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
    tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
    modification_list_[i] = tmp;
  }
  std::cout << "=>Calibration scale update done!\n";
}

void saveResult(const cv::Mat &calib_img, const int &frame_id) {
  std::string file_name = "calibration_" + std::to_string(frame_id) + ".txt";
  std::ofstream fCalib(file_name);
  if (!fCalib.is_open()) {
    std::cerr << "open file " << file_name << " failed." << std::endl;
    return;
  }
  fCalib << "Extrinsic:" << std::endl;
  fCalib << "R:\n"
         << calibration_matrix_(0, 0) << " " << calibration_matrix_(0, 1) << " "
         << calibration_matrix_(0, 2) << "\n"
         << calibration_matrix_(1, 0) << " " << calibration_matrix_(1, 1) << " "
         << calibration_matrix_(1, 2) << "\n"
         << calibration_matrix_(2, 0) << " " << calibration_matrix_(2, 1) << " "
         << calibration_matrix_(2, 2) << std::endl;
  fCalib << "t: " << calibration_matrix_(0, 3) << " "
         << calibration_matrix_(1, 3) << " " << calibration_matrix_(2, 3)
         << std::endl;
  fCalib << "\nIntrinsic:" << std::endl;
  fCalib << intrinsic_matrix_(0, 0) << " " << intrinsic_matrix_(0, 1) << " "
         << intrinsic_matrix_(0, 2) << "\n"
         << intrinsic_matrix_(1, 0) << " " << intrinsic_matrix_(1, 1) << " "
         << intrinsic_matrix_(1, 2) << "\n"
         << intrinsic_matrix_(2, 0) << " " << intrinsic_matrix_(2, 1) << " "
         << intrinsic_matrix_(2, 2) << std::endl;

  fCalib << "************* json format *************" << std::endl;
  fCalib << "Extrinsic:" << std::endl;
  fCalib << "[" << calibration_matrix_(0, 0) << "," << calibration_matrix_(0, 1)
         << "," << calibration_matrix_(0, 2) << "," << calibration_matrix_(0, 3)
         << "],"
         << "[" << calibration_matrix_(1, 0) << "," << calibration_matrix_(1, 1)
         << "," << calibration_matrix_(1, 2) << "," << calibration_matrix_(1, 3)
         << "],"
         << "[" << calibration_matrix_(2, 0) << "," << calibration_matrix_(2, 1)
         << "," << calibration_matrix_(2, 2) << "," << calibration_matrix_(2, 3)
         << "],"
         << "[" << calibration_matrix_(3, 0) << "," << calibration_matrix_(3, 1)
         << "," << calibration_matrix_(3, 2) << "," << calibration_matrix_(3, 3)
         << "]" << std::endl;
  fCalib << "\nIntrinsic:" << std::endl;
  fCalib << "[" << intrinsic_matrix_(0, 0) << "," << intrinsic_matrix_(0, 1)
         << "," << intrinsic_matrix_(0, 2) << "],"
         << "[" << intrinsic_matrix_(1, 0) << "," << intrinsic_matrix_(1, 1)
         << "," << intrinsic_matrix_(1, 2) << "],"
         << "[" << intrinsic_matrix_(2, 0) << "," << intrinsic_matrix_(2, 1)
         << "," << intrinsic_matrix_(2, 2) << "]" << std::endl;

  fCalib << "\nDistortion:" << std::endl;
  fCalib << "[";
  for (size_t i = 0; i < distortions_.size(); i++) {
    fCalib << distortions_[i];
    if (i == distortions_.size() - 1)
      continue;
    fCalib << ",";
  }
  fCalib << "]";
  fCalib.close();

  std::string img_name = "calibimg_" + std::to_string(frame_id) + ".jpg";
  cv::imwrite(img_name, calib_img);
}

// 加载3D框参数
std::vector<Box3D> LoadBoxes(const std::string& filename) {
  std::vector<Box3D> boxes;
  
  Json::Reader reader;
  Json::Value root;
  std::ifstream in(filename);
  
  if (!in.is_open()) {
    std::cout << "无法打开文件: " << filename << std::endl;
    return boxes;
  }
  
  if (!reader.parse(in, root, false)) {
    std::cout << "解析JSON失败: " << filename << std::endl;
    in.close();
    return boxes;
  }
  
  for (const auto& box_name : root.getMemberNames()) {
    Box3D box;
    box.name = box_name;
    
    Json::Value box_data = root[box_name];
    
    // 读取中心点
    box.center = Eigen::Vector3d(
      box_data["center"][0].asDouble(),
      box_data["center"][1].asDouble(),
      box_data["center"][2].asDouble()
    );
    
    // 读取方向向量
    box.x_dir = Eigen::Vector3d(
      box_data["x_direction"][0].asDouble(),
      box_data["x_direction"][1].asDouble(),
      box_data["x_direction"][2].asDouble()
    );
    
    box.y_dir = Eigen::Vector3d(
      box_data["y_direction"][0].asDouble(),
      box_data["y_direction"][1].asDouble(),
      box_data["y_direction"][2].asDouble()
    );
    
    box.z_dir = Eigen::Vector3d(
      box_data["z_direction"][0].asDouble(),
      box_data["z_direction"][1].asDouble(),
      box_data["z_direction"][2].asDouble()
    );
    
    // 读取尺寸
    box.size = Eigen::Vector3d(
      box_data["size"][0].asDouble(),
      box_data["size"][1].asDouble(),
      box_data["size"][2].asDouble()
    );
    
    boxes.push_back(box);
  }
  
  in.close();
  std::cout << "加载了 " << boxes.size() << " 个3D框" << std::endl;
  return boxes;
}

// 将3D点投射到2D图像
cv::Point2f project3DTo2D(const Eigen::Vector3d& point_3d, 
                          const Eigen::Matrix3d& K,
                          const Eigen::Matrix4d& extrinsic) {
  // 转换到相机坐标系
  Eigen::Vector4d point_homo(point_3d[0], point_3d[1], point_3d[2], 1.0);
  Eigen::Vector4d point_cam = extrinsic * point_homo;
  
  // 投射到图像平面
  Eigen::Vector3d point_2d_homo = K * point_cam.head<3>();
  
  if (point_2d_homo[2] > 0) {
    return cv::Point2f(point_2d_homo[0] / point_2d_homo[2], 
                       point_2d_homo[1] / point_2d_homo[2]);
  }
  
  return cv::Point2f(-1, -1);
}

// 绘制3D框到图像上
void drawBoxes3D(cv::Mat& img, 
                 const std::vector<Box3D>& boxes,
                 const Eigen::Matrix3d& K,
                 const Eigen::Matrix4d& extrinsic) {
  // 不同颜色用于不同的框
  std::vector<cv::Scalar> colors = {
    cv::Scalar(0, 255, 0),    // 绿色
    cv::Scalar(255, 0, 0),    // 蓝色
    cv::Scalar(0, 255, 255),  // 黄色
    cv::Scalar(255, 0, 255),  // 品红
    cv::Scalar(255, 255, 0),  // 青色
    cv::Scalar(128, 0, 128),  // 紫色
    cv::Scalar(255, 128, 0),  // 橙色
    cv::Scalar(0, 128, 255)   // 天蓝色
  };
  
  int color_idx = 0;
  for (const auto& box : boxes) {
    cv::Scalar color = colors[color_idx % colors.size()];
    color_idx++;
    
    // 获取8个顶点
    std::vector<Eigen::Vector3d> corners = box.getCorners();
    
    // 投射到2D
    std::vector<cv::Point2f> points_2d;
    bool all_valid = true;
    for (const auto& corner : corners) {
      cv::Point2f pt_2d = project3DTo2D(corner, K, extrinsic);
      
      // 检查点是否在图像范围内或深度为正
      if (pt_2d.x < 0 && pt_2d.y < 0) {
        all_valid = false;
        break;
      }
      points_2d.push_back(pt_2d);
    }
    
    if (!all_valid || points_2d.size() != 8) {
      continue;
    }
    
    // 绘制底面（顶点0-3）
    for (int i = 0; i < 4; i++) {
      cv::line(img, points_2d[i], points_2d[(i + 1) % 4], color, 2);
    }
    
    // 绘制顶面（顶点4-7）
    for (int i = 4; i < 8; i++) {
      cv::line(img, points_2d[i], points_2d[4 + (i + 1) % 4], color, 2);
    }
    
    // 绘制垂直边（连接底面和顶面）
    for (int i = 0; i < 4; i++) {
      cv::line(img, points_2d[i], points_2d[i + 4], color, 2);
    }
    
    // 在框上显示名称
    cv::putText(img, box.name, points_2d[0], cv::FONT_HERSHEY_SIMPLEX, 
                0.8, color, 2);
  }
}

bool ManualCalibration(int key_input) {
  char table[] = {'q', 'a', 'w', 's', 'e', 'd', 'r', 'f', 't', 'g', 'y', 'h'};
  bool real_hit = false;
  for (int32_t i = 0; i < 12; i++) {
    if (key_input == table[i]) {
      calibration_matrix_ = calibration_matrix_ * modification_list_[i];
      real_hit = true;
    }
  }
  // adjust fx, fy
  if (key_input == 'u') {
    intrinsic_matrix_(0, 0) *= cali_scale_fxfy_;
    std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
  }
  if (key_input == 'j') {
    intrinsic_matrix_(0, 0) /= cali_scale_fxfy_;
    std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
  }
  if (key_input == 'i') {
    intrinsic_matrix_(1, 1) *= cali_scale_fxfy_;
    std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
  }
  if (key_input == 'k') {
    intrinsic_matrix_(1, 1) /= cali_scale_fxfy_;
    std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
  }
  return real_hit;
}

int main(int argc, char **argv) {
  if (argc != 5 && argc != 6) {
    cout << "Usage: ./run_lidar2camera <image_path> <pcd_path> "
            "<intrinsic_json> <extrinsic_json> [box_json]"
            "\nexample:\n\t"
            "./bin/run_lidar2camera data/0.png data/0.pcd "
            "data/center_camera-intrinsic.json "
            "data/top_center_lidar-to-center_camera-extrinsic.json "
            "[data/box_parameters.json]"
         << endl;
    return 0;
  }

  string camera_path = argv[1];
  string lidar_path = argv[2];
  string intrinsic_json = argv[3];
  string extrinsic_json = argv[4];
  
  // 加载3D框（如果提供了box_json参数）
  std::vector<Box3D> boxes;
  string box_json = "";
  if (argc == 6) {
    box_json = argv[5];
    boxes = LoadBoxes(box_json);
  } else {
    // 尝试自动在相同目录下查找box_parameters.json
    boost::filesystem::path img_path(camera_path);
    boost::filesystem::path box_path = img_path.parent_path() / "box_parameters.json";
    if (boost::filesystem::exists(box_path)) {
      box_json = box_path.string();
      boxes = LoadBoxes(box_json);
    }
  }
  cv::Mat img = cv::imread(camera_path);
  std::cout << intrinsic_json << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(lidar_path, *cloud) == -1) {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  pcl::PointCloud<pcl::PointXYZI> pcd = *cloud;
  // load intrinsic
  Eigen::Matrix3d K;
  std::vector<double> dist;
  LoadIntrinsic(intrinsic_json, K, dist);
  for (size_t i = 0; i < dist.size(); i++) {
    distortions_.push_back(dist[i]);
  }

  intrinsic_matrix_ = K;
  orign_intrinsic_matrix_ = intrinsic_matrix_;
  std::cout << "intrinsic:\n"
            << K(0, 0) << " " << K(0, 1) << " " << K(0, 2) << "\n"
            << K(1, 0) << " " << K(1, 1) << " " << K(1, 2) << "\n"
            << K(2, 0) << " " << K(2, 1) << " " << K(2, 2) << "\n";
  std::cout << "dist:\n" << dist[0] << " " << dist[1] << "\n";

  // load extrinsic
  Eigen::Matrix4d json_param;
  LoadExtrinsic(extrinsic_json, json_param);

  cout << "Loading data completed!" << endl;
  CalibrationInit(json_param);

  std::cout << __LINE__ << "\n";

  Projector projector;
  projector.loadPointCloud(pcd);

  std::cout << __LINE__ << "\n";

  // view
  int width = img.cols;
  int height = img.rows;
  int panel_width = 200;  // 控制面板宽度
  int window_width = width + panel_width;  // 窗口总宽度 = 图片宽度 + 面板宽度
  std::cout << "image width:" << width << " , height:" << height << std::endl;
  std::cout << "window width:" << window_width << " , height:" << height << std::endl;
  pangolin::CreateWindowAndBind("lidar2camera player", window_width, height);

  glEnable(GL_DEPTH_TEST);
  // glDepthMask(GL_TRUE);
  // glDepthFunc(GL_LESS);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, 0.0, 1.0, 0.0));

  pangolin::View &project_image =
      pangolin::Display("project")
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(panel_width), 1.0)
          .SetLock(pangolin::LockLeft, pangolin::LockTop);

  unsigned char *imageArray = new unsigned char[3 * width * height];
  pangolin::GlTexture imageTexture(width, height, GL_RGB, false, 0, GL_RGB,
                                   GL_UNSIGNED_BYTE);

  // control panel
  pangolin::CreatePanel("cp").SetBounds(pangolin::Attach::Pix(30), 1.0, 0.0,
                                        pangolin::Attach::Pix(panel_width));
  pangolin::Var<bool> displayMode("cp.Intensity Color", false,
                                  true);                            // logscale
  pangolin::Var<bool> filterMode("cp.Overlap Filter", false, true); // logscale
  pangolin::Var<bool> showBoxes("cp.Show 3D Boxes", !boxes.empty(), true); // 显示3D框
  pangolin::Var<double> degreeStep("cp.deg step", 0.3, 0, 1);       // logscale
  pangolin::Var<double> tStep("cp.t step(cm)", 6, 0, 15);
  pangolin::Var<double> fxfyScale("cp.fxfy scale", 1.005, 1, 1.1);
  pangolin::Var<int> pointSize("cp.point size", 3, 1, 5);

  pangolin::Var<bool> addXdegree("cp.+ x degree", false, false);
  pangolin::Var<bool> minusXdegree("cp.- x degree", false, false);
  pangolin::Var<bool> addYdegree("cp.+ y degree", false, false);
  pangolin::Var<bool> minusYdegree("cp.- y degree", false, false);
  pangolin::Var<bool> addZdegree("cp.+ z degree", false, false);
  pangolin::Var<bool> minusZdegree("cp.- z degree", false, false);
  pangolin::Var<bool> addXtrans("cp.+ x trans", false, false);
  pangolin::Var<bool> minusXtrans("cp.- x trans", false, false);
  pangolin::Var<bool> addYtrans("cp.+ y trans", false, false);
  pangolin::Var<bool> minusYtrans("cp.- y trans", false, false);
  pangolin::Var<bool> addZtrans("cp.+ z trans", false, false);
  pangolin::Var<bool> minusZtrans("cp.- z trans", false, false);

  pangolin::Var<bool> addFx("cp.+ fx", false, false);
  pangolin::Var<bool> minusFx("cp.- fx", false, false);
  pangolin::Var<bool> addFy("cp.+ fy", false, false);
  pangolin::Var<bool> minusFy("cp.- fy", false, false);

  pangolin::Var<bool> resetButton("cp.Reset", false, false);
  pangolin::Var<bool> saveImg("cp.Save Image", false, false);

  std::vector<pangolin::Var<bool>> mat_calib_box;
  mat_calib_box.push_back(addXdegree);
  mat_calib_box.push_back(minusXdegree);
  mat_calib_box.push_back(addYdegree);
  mat_calib_box.push_back(minusYdegree);
  mat_calib_box.push_back(addZdegree);
  mat_calib_box.push_back(minusZdegree);
  mat_calib_box.push_back(addXtrans);
  mat_calib_box.push_back(minusXtrans);
  mat_calib_box.push_back(addYtrans);
  mat_calib_box.push_back(minusYtrans);
  mat_calib_box.push_back(addZtrans);
  mat_calib_box.push_back(minusZtrans);

  cv::Mat current_frame = projector.ProjectToRawImage(
      img, intrinsic_matrix_, dist, calibration_matrix_);
  
  // 绘制3D框
  if (!boxes.empty()) {
    drawBoxes3D(current_frame, boxes, intrinsic_matrix_, calibration_matrix_);
  }
  
  int frame_num = 0;
  bool show_boxes_mode = !boxes.empty();

  std::cout << "\n=>START\n";
  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (displayMode) {
      if (display_mode_ == false) {
        projector.setDisplayMode(true);
        current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
                                                    dist, calibration_matrix_);
        if (showBoxes && !boxes.empty()) {
          drawBoxes3D(current_frame, boxes, intrinsic_matrix_, calibration_matrix_);
        }
        display_mode_ = true;
      }
    } else {
      if (display_mode_ == true) {
        projector.setDisplayMode(false);
        current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
                                                    dist, calibration_matrix_);
        if (showBoxes && !boxes.empty()) {
          drawBoxes3D(current_frame, boxes, intrinsic_matrix_, calibration_matrix_);
        }
        display_mode_ = false;
      }
    }

    if (filterMode) {
      if (filter_mode_ == false) {
        projector.setFilterMode(true);
        current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
                                                    dist, calibration_matrix_);
        if (showBoxes && !boxes.empty()) {
          drawBoxes3D(current_frame, boxes, intrinsic_matrix_, calibration_matrix_);
        }
        filter_mode_ = true;
      }
    } else {
      if (filter_mode_ == true) {
        projector.setFilterMode(false);
        current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
                                                    dist, calibration_matrix_);
        if (showBoxes && !boxes.empty()) {
          drawBoxes3D(current_frame, boxes, intrinsic_matrix_, calibration_matrix_);
        }
        filter_mode_ = false;
      }
    }
    
    // 处理3D框显示开关
    if (showBoxes) {
      if (show_boxes_mode == false && !boxes.empty()) {
        current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
                                                    dist, calibration_matrix_);
        drawBoxes3D(current_frame, boxes, intrinsic_matrix_, calibration_matrix_);
        show_boxes_mode = true;
      }
    } else {
      if (show_boxes_mode == true) {
        current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
                                                    dist, calibration_matrix_);
        show_boxes_mode = false;
      }
    }

    if (degreeStep.GuiChanged()) {
      cali_scale_degree_ = degreeStep.Get();
      CalibrationScaleChange();
      std::cout << "Degree calib scale changed to " << cali_scale_degree_
                << " degree\n";
    }
    if (tStep.GuiChanged()) {
      cali_scale_trans_ = tStep.Get() / 100.0;
      CalibrationScaleChange();
      std::cout << "Trans calib scale changed to " << cali_scale_trans_ * 100
                << " cm\n";
    }
    if (fxfyScale.GuiChanged()) {
      cali_scale_fxfy_ = fxfyScale.Get();
      std::cout << "fxfy calib scale changed to " << cali_scale_fxfy_
                << std::endl;
    }
    if (pointSize.GuiChanged()) {
      int ptsize = pointSize.Get();
      projector.setPointSize(ptsize);
      current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                                  calibration_matrix_);
      if (showBoxes && !boxes.empty()) {
        drawBoxes3D(current_frame, boxes, intrinsic_matrix_, calibration_matrix_);
      }
      std::cout << "point size changed to " << ptsize << std::endl;
    }
    for (int i = 0; i < 12; i++) {
      if (pangolin::Pushed(mat_calib_box[i])) {
        calibration_matrix_ = calibration_matrix_ * modification_list_[i];
        std::cout << "Changed!\n";
        current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
                                                    dist, calibration_matrix_);
        if (showBoxes && !boxes.empty()) {
          drawBoxes3D(current_frame, boxes, intrinsic_matrix_, calibration_matrix_);
        }
      }
    }

    if (pangolin::Pushed(addFx)) {
      intrinsic_matrix_(0, 0) *= cali_scale_fxfy_;
      current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                                  calibration_matrix_);
      if (showBoxes && !boxes.empty()) {
        drawBoxes3D(current_frame, boxes, intrinsic_matrix_, calibration_matrix_);
      }
      std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
    }
    if (pangolin::Pushed(minusFx)) {
      intrinsic_matrix_(0, 0) /= cali_scale_fxfy_;
      current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                                  calibration_matrix_);
      if (showBoxes && !boxes.empty()) {
        drawBoxes3D(current_frame, boxes, intrinsic_matrix_, calibration_matrix_);
      }
      std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
    }
    if (pangolin::Pushed(addFy)) {
      intrinsic_matrix_(1, 1) *= cali_scale_fxfy_;
      current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                                  calibration_matrix_);
      if (showBoxes && !boxes.empty()) {
        drawBoxes3D(current_frame, boxes, intrinsic_matrix_, calibration_matrix_);
      }
      std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
    }
    if (pangolin::Pushed(minusFy)) {
      intrinsic_matrix_(1, 1) /= cali_scale_fxfy_;
      current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                                  calibration_matrix_);
      if (showBoxes && !boxes.empty()) {
        drawBoxes3D(current_frame, boxes, intrinsic_matrix_, calibration_matrix_);
      }
      std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
    }

    if (pangolin::Pushed(resetButton)) {
      calibration_matrix_ = orign_calibration_matrix_;
      intrinsic_matrix_ = orign_intrinsic_matrix_;
      current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                                  calibration_matrix_);
      if (showBoxes && !boxes.empty()) {
        drawBoxes3D(current_frame, boxes, intrinsic_matrix_, calibration_matrix_);
      }
      std::cout << "Reset!\n";
    }
    if (pangolin::Pushed(saveImg)) {
      saveResult(current_frame, frame_num);
      std::cout << "\n==>Save Result " << frame_num << std::endl;
      Eigen::Matrix4d transform = calibration_matrix_;
      cout << "Transfromation Matrix:\n" << transform << std::endl;
      frame_num++;
    }

    if (kbhit()) {
      int c = getchar();
      if (ManualCalibration(c)) {
        Eigen::Matrix4d transform = calibration_matrix_;
        cout << "\nTransfromation Matrix:\n" << transform << std::endl;
      }
      current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                                  calibration_matrix_);
      if (showBoxes && !boxes.empty()) {
        drawBoxes3D(current_frame, boxes, intrinsic_matrix_, calibration_matrix_);
      }
    }

    imageArray = current_frame.data;
    imageTexture.Upload(imageArray, GL_BGR, GL_UNSIGNED_BYTE);

    project_image.Activate();
    glColor3f(1.0, 1.0, 1.0);
    imageTexture.RenderToViewportFlipY();

    pangolin::FinishFrame();
    glFinish();
  }

  // delete[] imageArray;

  Eigen::Matrix4d transform = calibration_matrix_;
  cout << "\nFinal Transfromation Matrix:\n" << transform << std::endl;

  return 0;
}
