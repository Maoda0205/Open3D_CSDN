#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <memory>
#include <math.h>

#include "open3d/Open3D.h"

using namespace std;
using namespace open3d;


int main(int argc, char *argv[]) {
    // Point cloud outlier removal
      string pointcloud_path ="../data/fragment.pcd";  //读取文件的地址

      std::shared_ptr<geometry::PointCloud> pointcloud = io::CreatePointCloudFromFile(pointcloud_path);

      if (pointcloud == nullptr) {
          utility::LogWarning("Unable to load pointcloud file.");
          return -1;
      }; //检查文件是否为空

      //RemoveStatisticalOutliers方法
      Eigen::Vector3d color(1.0,0,0);//将所有点设置为红色

      pointcloud->open3d::geometry::PointCloud::PaintUniformColor(color);//把整个点云设置为红色

      auto downsampled = pointcloud->VoxelDownSample(0.03);//对点云进行下采样

      auto inlier =std::get<0>(downsampled->RemoveStatisticalOutliers(30,1.0)); //0返回正常值点云
      auto indices_vec =std::get<1>(downsampled->RemoveStatisticalOutliers(30,1.0));  //1返回正常值的索引vector

      for (size_t i = 0; i < indices_vec.size(); i++) {

              downsampled->colors_[indices_vec[i]] =
                      Eigen::Vector3d(0.5,0.5,0.5);  //正常值设置成灰色
          }

      visualization::DrawGeometries({downsampled}, "RemoveStatisticalOutliers", 1600, 900);


      //RemoveRadiusOutliers方法

      Eigen::Vector3d color1(0.5,0.5,0.5);//将所有点设置为灰色

      pointcloud->open3d::geometry::PointCloud::PaintUniformColor(color1);//把整个点云设置为灰色

      auto downsampled1 = pointcloud->VoxelDownSample(0.03);//对点云进行下采样

      auto inlier1 =std::get<0>(downsampled1->RemoveRadiusOutliers(16,0.09)); //0返回正常值点云

      visualization::DrawGeometries({inlier1}, "RemoveRadiusOutliers", 1600, 900);

     return 0;

}

