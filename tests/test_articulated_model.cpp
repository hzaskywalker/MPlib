#include <articulated_model.h>
#include <Eigen/Core>

using ArticulatedModel = ArticulatedModelTpl<double>;

int main() {
  // set up an articulated model
  std::string urdf_filename = "../data/panda/panda.urdf";
  std::string srdf_filename = "../data/panda/panda.srdf";
  Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.81);
  ArticulatedModel articulated_model(urdf_filename, srdf_filename, gravity, {}, {}, false, false);
}
