#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <memory>
#include <math.h>

#include "open3d/Open3D.h"

using namespace std;
using namespace open3d;

bool f_traverse(const std::shared_ptr<geometry::OctreeNode>& node,
                const std::shared_ptr<geometry::OctreeNodeInfo>& node_info) {
    if (auto internal_node =
                std::dynamic_pointer_cast<geometry::OctreeInternalNode>(node)) {
        if (auto internal_point_node = std::dynamic_pointer_cast<
                    geometry::OctreeInternalPointNode>(internal_node)) {
            int num_children = 0;
            for (const auto& c : internal_point_node->children_) {
                if (c) num_children++;
            }
            utility::LogInfo(
                    "Internal node at depth {} with origin {} has "
                    "{} children and {} points",
                    node_info->depth_, node_info->origin_, num_children,
                    internal_point_node->indices_.size());
        }
    } else if (auto leaf_node = std::dynamic_pointer_cast<
                       geometry::OctreePointColorLeafNode>(node)) {
        utility::LogInfo(
                "Node at depth {} with origin {} has"
                "color {} and {} points",
                node_info->depth_, node_info->origin_, leaf_node->color_,
                leaf_node->indices_.size());
        // utility::LogInfo("Indices: {}", leaf_node->indices_);
    } else {
        utility::LogError("Unknown node type");
    }

    return false;
}

int main(int argc, char *argv[]) {
         auto pcd = io::CreatePointCloudFromFile("../data/fragment.ply");
        visualization::DrawGeometries({pcd},"input");

        constexpr int max_depth = 4; //设置最大深度为4
        auto octree = std::make_shared<geometry::Octree>(max_depth);
        octree->ConvertFromPointCloud(*pcd);

        octree->Traverse(f_traverse);

        std::cout << std::endl << std::endl;
        auto start = std::chrono::steady_clock::now();
        auto result = octree->LocateLeafNode(Eigen::Vector3d::Zero());
        auto end = std::chrono::steady_clock::now();
        utility::LogInfo(
                "Located in {} usec",
                std::chrono::duration_cast<std::chrono::microseconds>(end - start)
                        .count());
        if (auto point_node =
                    std::dynamic_pointer_cast<geometry::OctreePointColorLeafNode>(
                            result.first)) {
            utility::LogInfo(
                    "Found leaf node at depth {} with origin {} and {} indices",
                    result.second->depth_, result.second->origin_,
                    point_node->indices_.size());
        }
        std::cout << std::endl << std::endl;

        visualization::DrawGeometries({pcd, octree},"octree output");
        return 0;
}
