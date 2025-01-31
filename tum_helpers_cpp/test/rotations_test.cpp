#include "tum_helpers_cpp/rotations.hpp"
#include "matplotlib_cpp/matplotlibcpp.hpp"

namespace plt = matplotlibcpp;

int main(){
Eigen::VectorXd roll = 1.0 / 180.0 * 3.14 * Eigen::VectorXd::LinSpaced(90,-20,20);
Eigen::VectorXd pitch = Eigen::VectorXd::Zero(roll.size());
Eigen::VectorXd ay_3d = 5.0 * Eigen::VectorXd::Ones(roll.size());
Eigen::VectorXd az_3d = 9.81 * Eigen::VectorXd::Ones(roll.size());
Eigen::VectorXd ax_3d = Eigen::VectorXd::Zero(roll.size());
tam::helpers::euler_rotations::Eigen3D a_3d{ax_3d, ay_3d, az_3d};

tam::helpers::euler_rotations::Eigen3D a_2d = tam::helpers::euler_rotations::vector_to_2d(a_3d, pitch, roll);

Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(roll.size(),0,roll.size());

Eigen::VectorXd a_2d_ref = (ay_3d.array() * roll.array().cos()).matrix() - (az_3d.array() * roll.array().sin()).matrix(); 
Eigen::VectorXd sin_roll = (roll.array().sin()).matrix();

plt::figure();
plt::plot(x, a_2d.y, "-", {{"label", "tum_helpers_trafo"}});
plt::plot(x, a_2d_ref, "-", {{"label", "manual_trafo"}});
plt::legend();
plt::xlabel("banking in degree");
plt::ylabel("a_y_2d");
plt::show();

return 0;
}
