#include <Eigen/Dense>
#include <iostream>
#include <memory>

#include "open3d/Open3D.h"

using namespace open3d;

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
            open3d::io::CreatePointCloudFromFile("../data/frag_115.ply");
    std::shared_ptr<geometry::PointCloud> target =
            open3d::io::CreatePointCloudFromFile("../data/frag_116.ply");
    if (source == nullptr || target == nullptr) {
        utility::LogWarning("Unable to load source or target file.");
        return -1;
    }
    visualization::DrawGeometries({source, target},"initial state");

    std::vector<double> voxel_sizes = {0.05, 0.05 / 4};
    std::vector<int> iterations = {50,14};
    Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 2; ++i) {
        float voxel_size = voxel_sizes[i];

        auto source_down = source->VoxelDownSample(voxel_size);
        source_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
                voxel_size * 2.0, 30));

        auto target_down = target->VoxelDownSample(voxel_size);
        target_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
                voxel_size * 2.0, 30));


        auto result = pipelines::registration::RegistrationColoredICP(
                *source_down, *target_down, 0.07, trans,
                pipelines::registration::
                        TransformationEstimationForColoredICP(),
                pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6,
                                                                iterations[i]));
        trans = result.transformation_;

            VisualizeRegistration(*source, *target, trans);
            std::cout<<"匹配方式：Colored_ICP"<<" "<<"inlier_rmse:"<<result.inlier_rmse_<<std::endl;

    }

    std::stringstream ss;
    ss << trans;
    utility::LogInfo("Final transformation = \n{}", ss.str());

    return 0;
}
