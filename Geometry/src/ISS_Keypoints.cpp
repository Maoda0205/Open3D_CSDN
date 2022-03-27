#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <memory>
#include <math.h>

#include "open3d/Open3D.h"

#define M_PI 3.1415926

using namespace std;
using namespace open3d;

int main(int argc, char *argv[]) {

            const std::string option("mesh"); //选择输入文件的类型mesh/pointclooud
            const std::string filename("../data/ArmadilloMesh.ply"); //选择输入的路径
            auto cloud = std::make_shared<geometry::PointCloud>();
            auto mesh = std::make_shared<geometry::TriangleMesh>();
            if (option == "mesh") {
                if (!io::ReadTriangleMesh(filename, *mesh)) {
                    utility::LogWarning("Failed to read {}", filename);
                    return 1;
                }
                cloud->points_ = mesh->vertices_;
            } else if (option == "pointcloud") {
                if (!io::ReadPointCloud(filename, *cloud)) {
                    utility::LogWarning("Failed to read {}\n\n", filename);
                    return 1;
                }
            } else {
                utility::LogError("option {} not supported\n", option);
            }

            // 计算ISS Keypoints
            auto iss_keypoints = std::make_shared<geometry::PointCloud>();
            {
                utility::ScopeTimer timer("ISS Keypoints estimation"); //计算时间
                iss_keypoints = geometry::keypoint::ComputeISSKeypoints(*cloud);
                utility::LogInfo("Detected {} keypoints",
                                 iss_keypoints->points_.size());
            }

            // 展示结果
            cloud->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
            iss_keypoints->PaintUniformColor(Eigen::Vector3d(1.0, 0.75, 0.0));
            if (option == "mesh") {
                mesh->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
                mesh->ComputeVertexNormals();
                mesh->ComputeTriangleNormals();
                visualization::DrawGeometries({mesh, iss_keypoints}, "ISS", 1600, 900);
            } else {
                visualization::DrawGeometries({iss_keypoints}, "ISS", 1600, 900);
            }

     return 0;

}

