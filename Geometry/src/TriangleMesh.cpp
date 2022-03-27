#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <memory>
#include <math.h>

#include "open3d/Open3D.h"

using namespace std;
using namespace open3d;    

int main(int argc, char *argv[]) {
	string BunnyMesh_path ="../data/BunnyMesh.ply";  //读取文件的地址

	std::shared_ptr<geometry::TriangleMesh> BunnyMesh(new geometry::TriangleMesh);

	BunnyMesh = io::CreateMeshFromFile(BunnyMesh_path);

	BunnyMesh->ComputeVertexNormals();

	visualization::DrawGeometries({BunnyMesh});

	return 0;

  }
