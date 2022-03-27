#include <Eigen/Dense>
#include <iostream>
#include <memory>

#include "open3d/Open3D.h"

using namespace open3d;
using namespace std;

void VisualizeRegistration(const open3d::geometry::PointCloud &source,
                           const open3d::geometry::PointCloud &target,
                           const Eigen::Matrix4d &Transformation) {
    std::shared_ptr<geometry::PointCloud> source_transformed_ptr(
            new geometry::PointCloud);
    std::shared_ptr<geometry::PointCloud> target_ptr(new geometry::PointCloud);
    *source_transformed_ptr = source;
    *target_ptr = target;
    source_transformed_ptr->Transform(Transformation);
    visualization::DrawGeometries({source_transformed_ptr, target_ptr},
                                  "Registration result");
}


int main(int argc, char *argv[]) {
    using namespace open3d;

    // Prepare input
    std::shared_ptr<geometry::PointCloud> source =
            open3d::io::CreatePointCloudFromFile("../data/fragment.ply");
    std::shared_ptr<geometry::PointCloud> target =
            open3d::io::CreatePointCloudFromFile("../data/fragment2.ply");
    if (source == nullptr || target == nullptr) {
        utility::LogWarning("Unable to load source or target file.");
        return -1;
    }

    visualization::DrawGeometries({source, target},
                                  "initial state");

    std::vector<double> voxel_sizes = {0.05, 0.05 / 4}; //下采样体素栅格的边长
    std::vector<int> iterations = {50, 14}; //最大迭代次数
    Eigen::Matrix4d trans_point2point= Eigen::Matrix4d::Identity();
    Eigen::Matrix4d trans_point2plane = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 2; ++i) {
        float voxel_size = voxel_sizes[i];

        auto source_down = source->VoxelDownSample(voxel_size);
        source_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
                voxel_size * 2.0, 30));

        auto target_down = target->VoxelDownSample(voxel_size);
        target_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
                voxel_size * 2.0, 30));

        double max_correspondence_distance=0.07;  //对应点对的最大距离，这个值对于结果影响很大

        auto result_point2point = pipelines::registration::RegistrationICP( //点到点的ICP
                *source_down, *target_down, max_correspondence_distance, trans_point2point,
                pipelines::registration::
                        TransformationEstimationPointToPoint(),
                pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6,
                                                                iterations[i]));

        auto result_point2plane = pipelines::registration::RegistrationICP( //点到面的ICP
                *source_down, *target_down, max_correspondence_distance, trans_point2plane,
                pipelines::registration::
                        TransformationEstimationPointToPlane(),
                pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6,
                                                                iterations[i]));

        trans_point2point = result_point2point.transformation_;

        trans_point2plane = result_point2plane.transformation_;

        cout<<"匹配方式：点对点"<<" "<<"inlier_rmse:"<<result_point2point.inlier_rmse_<<endl;
        VisualizeRegistration(*source, *target, trans_point2point); //显示配准结果

        cout<<"匹配方式：点对面"<<" "<<"inlier_rmse:"<<result_point2plane.inlier_rmse_<<endl;;
        VisualizeRegistration(*source, *target, trans_point2plane); //显示配准结果

    }

      return 0;
}
