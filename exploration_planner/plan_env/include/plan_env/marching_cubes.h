#ifndef MESH_H
#define MESH_H

#include <pcl/geometry/triangle_mesh.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <plan_env/marching_cubes.h>
#include <iostream>
#include <string>

namespace fast_planner {
struct Cell {
  pcl::PointXYZ vert[8];
  double val[8];
};

class MarchingCubes {
 public:
  MarchingCubes() {
    count_ = 0;
    isolevel_ = 0.1;
  }
  ~MarchingCubes() {}

  void setParam(const int isolevel) { isolevel_ = isolevel; }
  // Calculate the intersection on an edge
  pcl::PointXYZ VertexInterp(double isolevel, pcl::PointXYZ p1, pcl::PointXYZ p2, double valp1,
                             double valp2);
  int process_cube(Cell grid, pcl::PointCloud<pcl::PointXYZ> &cloud);
  void generateMeshFile(const std::vector<std::vector<std::vector<double>>> &tsdfGrid,
                        const std::string &mesh_file_name, const int x_max, const int y_max,
                        const int z_max);

 private:
  int count_;
  double isolevel_;
};
}  // namespace fast_planner
#endif
