#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <memory>

#include "open3d/Open3D.h"

using namespace std;
using namespace open3d;

int main(int argc, char *argv[]) {
    string pointcloud_path ="../data/fragment.ply";  //读取文件的地址

    std::shared_ptr<geometry::PointCloud> pointcloud = 
                          io::CreatePointCloudFromFile(pointcloud_path);

    if (pointcloud == nullptr) {
        utility::LogWarning("Unable to load source or target file.");
        return -1;
    }; //检查文件是否为空

    // test downsample
    auto downsampled = pointcloud->VoxelDownSample(0.05);
    visualization::DrawGeometries({downsampled}, "Down Sampled Pointcloud");
    
    return 0;
}
