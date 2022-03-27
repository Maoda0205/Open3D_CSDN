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

        //Translate
        auto input = io::CreatePointCloudFromFile("../data/fragment.pcd");
        auto input_tx = io::CreatePointCloudFromFile("../data/fragment.pcd");
        auto input_ty = io::CreatePointCloudFromFile("../data/fragment.pcd");
        Eigen::Vector3d tx(1.5,0,0);
        Eigen::Vector3d ty(0,1.5,0);

        input_tx->Translate(tx);
        input_ty->Translate(ty);

        visualization::DrawGeometries({input,input_tx,input_ty},"Translate output");

        //rotate
        auto input_rotate = io::CreatePointCloudFromFile("../data/fragment.pcd");

        Eigen::Vector3d rotate1(M_PI/2,0,M_PI/4);
        Eigen::Vector3d center(0,0,0);

        auto R =open3d::geometry::PointCloud::GetRotationMatrixFromXYZ(rotate1);

        input_rotate->Rotate(R,center);
        visualization::DrawGeometries({input,input_rotate},"Rotate output");

        //Transform
        auto input_Transform = io::CreatePointCloudFromFile("../data/fragment.pcd");

        Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity(); //4*4的单位阵
        transformation.block<3, 3>(0, 0) = static_cast<Eigen::Matrix3d>(
                Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitX()));
        input_Transform->Transform(transformation);
        visualization::DrawGeometries({input,input_Transform},"Transform output");

        return 0;

}

