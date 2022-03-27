#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <memory>
#include <math.h>

#include "open3d/Open3D.h"

using namespace std;
using namespace open3d;

int main(int argc, char *argv[]) {

    string pointcloud_path ="../data/fragment.ply";  //读取文件的地址

    std::shared_ptr<geometry::PointCloud> pointcloud = io::CreatePointCloudFromFile(pointcloud_path);

    if (pointcloud == nullptr) {
        utility::LogWarning("Unable to load source or target file.");
        return -1;
    }; //检查文件是否为空

    auto downsampled = pointcloud->VoxelDownSample(0.05);

    visualization::DrawGeometriesWithKeyCallbacks(
            {downsampled},
            {{GLFW_KEY_SPACE,
              [&](visualization::Visualizer *vis) {
                    downsampled->EstimateNormals(
                          open3d::geometry::KDTreeSearchParamKNN(30));
//                  downsampled->EstimateNormals(
//                            open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
//                  downsampled->EstimateNormals(
//                          open3d::geometry::KDTreeSearchParamRadius(0.1));
                  utility::LogInfo("Done.");
                  return true;
              }}},
            "Press Space to Estimate Normal", 1600, 900);

    cout<<downsampled->normals_[0]<<endl;//打印第 0 个点的法线向量

    return 0;
}
