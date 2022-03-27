#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <memory>

#include "open3d/Open3D.h"

using namespace std;
using namespace open3d;

int main(int argc, char *argv[]) {

    //******Visualize point cloud******//
    visualization::Visualizer visualizer;

    string pointcloud_path ="../data/fragment.ply";  //文件的地址

    std::shared_ptr<geometry::PointCloud> pointcloud = 
                       io::CreatePointCloudFromFile(pointcloud_path);

    if (pointcloud == nullptr) {
        utility::LogWarning("Unable to load pointcloud file.");
        return -1;
    };  //检查文件是否为空

    visualizer.CreateVisualizerWindow("Open3D", 1600, 900);
    visualizer.AddGeometry(pointcloud);
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();

    bool result=open3d::io::WritePointCloudToPLY("../data/fragment_save.ply",*pointcloud,false);

    if(result==1){std::cout<<"保存成功"<<std::endl;}


    return 0;
}
