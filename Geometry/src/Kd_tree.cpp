#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <memory>
#include <math.h>

#include "open3d/Open3D.h"

using namespace std;
using namespace open3d;


int main(int argc, char *argv[]) {
    auto new_cloud_ptr = std::make_shared<geometry::PointCloud>();//读入点云数据，并判断是否读入成功
    if (io::ReadPointCloud("../data/fragment.ply", *new_cloud_ptr)) {
        utility::LogInfo("Successfully read {}", "../data/fragment.ply");
    } else {
        utility::LogWarning("Failed to read {}", "../data/fragment.ply");
        return 1;
    }

    Eigen::Vector3d color(0.5,0.5,0.5); //把点云设置为灰色

    new_cloud_ptr->open3d::geometry::PointCloud::PaintUniformColor(color);//把点云设置为灰色

    //最大个数的搜索方法
    geometry::KDTreeFlann kdtree;
    int num =200; //最近邻个数
    kdtree.SetGeometry(*new_cloud_ptr);
    std::vector<int> new_indices_vec(num);//最近邻对应的点云索引序号
    std::vector<double> new_dists_vec(num);//最近邻离中心点的距离
    kdtree.SearchKNN(new_cloud_ptr->points_[1500], num, new_indices_vec, //将第1500个点设置为搜索的中心点
                     new_dists_vec);

    for (size_t i = 0; i < new_indices_vec.size(); i++) {
        utility::LogInfo("{:d}, {:f}", (int)new_indices_vec[i],
                         sqrt(new_dists_vec[i]));
        new_cloud_ptr->colors_[new_indices_vec[i]] =
                Eigen::Vector3d(1.0, 0.0, 0.0);  //最近邻图成红色
    }

     new_cloud_ptr->colors_[1500] = Eigen::Vector3d(0.0, 1.0, 0.0); //搜索中心设置为绿色

     //最大半径的搜索方法
    float r = 0.1;
    int k = kdtree.SearchRadius(new_cloud_ptr->points_[4000], r, new_indices_vec,
                                new_dists_vec);

    utility::LogInfo("======== {:d}, {:f} ========", k, r);
    for (int i = 0; i < k; i++) {
        utility::LogInfo("{:d}, {:f}", (int)new_indices_vec[i],
                         sqrt(new_dists_vec[i]));
        new_cloud_ptr->colors_[new_indices_vec[i]] =
                Eigen::Vector3d(0.0, 0.0, 1.0);//最近邻图成蓝色
    }
    new_cloud_ptr->colors_[4000] = Eigen::Vector3d(1.0, 0.0, 0.0);//搜索中心设置为红色


    visualization::DrawGeometries({new_cloud_ptr}, "KDTreeFlann", 1600, 900);

     return 0;

}

