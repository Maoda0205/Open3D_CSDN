#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <memory>
#include <math.h>

#include "open3d/Open3D.h"

using namespace std;
using namespace open3d;

std::shared_ptr<geometry::RGBDImage> ReadRGBDImage(
        const char* color_filename,
        const char* depth_filename,
        const camera::PinholeCameraIntrinsic& intrinsic,
        bool visualize) {
    geometry::Image color, depth;
    io::ReadImage(color_filename, color);
    io::ReadImage(depth_filename, depth);
    utility::LogInfo("Reading RGBD image : ");
    utility::LogInfo("     Color : {:d} x {:d} x {:d} ({:d} bits per channel)",
                     color.width_, color.height_, color.num_of_channels_,
                     color.bytes_per_channel_ * 8);
    utility::LogInfo("     Depth : {:d} x {:d} x {:d} ({:d} bits per channel)",
                     depth.width_, depth.height_, depth.num_of_channels_,
                     depth.bytes_per_channel_ * 8);
    double depth_scale = 1000.0, depth_trunc = 3.0;
    bool convert_rgb_to_intensity = true;
    std::shared_ptr<geometry::RGBDImage> rgbd_image =
            geometry::RGBDImage::CreateFromColorAndDepth(
                    color, depth, depth_scale, depth_trunc,
                    convert_rgb_to_intensity);
    if (visualize) {
        auto pcd = geometry::PointCloud::CreateFromRGBDImage(*rgbd_image,
                                                             intrinsic);
        visualization::DrawGeometries({pcd});
    }
    return rgbd_image;
}

int main(int argc, char *argv[]) {

    camera::PinholeCameraIntrinsic intrinsic = camera::PinholeCameraIntrinsic(
            camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);
    bool visualize = true;

    auto source = ReadRGBDImage("../data/RGBD/color/00000.jpg", "../data/RGBD/depth/00000.png", intrinsic, visualize); //读取RGBD图
    auto target = ReadRGBDImage("../data/RGBD/color/00001.jpg", "../data/RGBD/depth/00001.png", intrinsic, visualize); //读取RGBD图

    Eigen::Matrix4d odo_init = Eigen::Matrix4d::Identity();
    std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d> rgbd_odo =
            pipelines::odometry::ComputeRGBDOdometry(
                    *source, *target, intrinsic, odo_init,
                    pipelines::odometry::RGBDOdometryJacobianFromHybridTerm(),
                    pipelines::odometry::OdometryOption());
    std::cout << "RGBD Odometry" << std::endl;
    std::cout << std::get<1>(rgbd_odo) << std::endl;

    return 0;
}
