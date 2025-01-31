// Copyright 2023 Simon Hoffmann
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <string>
#include <tuple>

#include "track_handler_cpp/acceleration.hpp"
#include "track_handler_cpp/track.hpp"
#include "tum_helpers_cpp/test_helpers/test_helpers.hpp"
class acceleration_tests_vegas : public ::testing::Test
{
public:
  acceleration_tests_vegas()
  {
    std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("track_handler_cpp");

    th = tam::common::Track::create_from_csv(
      package_share_directory + "/test_data/tracks/LVMS_3d_smoothed.csv");

    s.resize(3, 4);
    s << 569.0918470146787, 572.2350984248989, 575.4002137887089, 578.576777688324,  //
      569.0918470146787, 572.2350984248989, 575.4002137887089, 578.576777688324,     //
      569.0918470146787, 572.2350984248989, 575.4002137887089, 578.576777688324;

    ax.resize(3, 4);
    ax << 4.961954091287942, 3.2941776700939633, 1.702389910853892, 0.3467853924135433,  //
      4.961954091287942, 3.293932014847385, 1.7010944295105, 0.34369812842080844,        //
      4.961954091287942, 3.2936935025941834, 1.699850161043574, 0.34076595411321736;

    ay.resize(3, 4);
    ay << 6.149339649618665, 6.15026950598575, 6.117288158680642, 6.0500872708279445,  //
      6.149339649618665, 6.208221693006897, 6.226070308011797, 6.203279100818871,      //
      6.149339649618665, 6.266174034080363, 6.334853583570468, 6.356475820495719;

    chi.resize(3, 4);
    chi << 0.002648595731858867, 0.0025117107325265905, 0.002131844712251059, 0.001539873550483828,
      0.002648595731858867, 0.002573234272893615, 0.0023667926633845504, 0.0020456670403088727,
      0.002648595731858867, 0.0026347579343620874, 0.002601741745209191, 0.0025514672931942384;

    ax_tilde.resize(3, 4);
    ax_tilde << 4.949465428509238, 3.2839416391632903, 1.6952606982162373, 0.3435294731593947,
      4.949465428509238, 3.283489555806497, 1.693176915290537, 0.33874516695455387,  //
      4.949465428509238, 3.2830446150762795, 1.691144341884642, 0.33411592901051357;

    ay_tilde.resize(3, 4);
    ay_tilde << 2.793982000179578, 2.794998848840569, 2.7620693430848964, 2.6948786034974126,
      2.793982000179578, 2.8529516719686727, 2.8708532600148833, 2.848072509487597,  //
      2.793982000179578, 2.9109046618505716, 2.9796384883920197, 3.0012721635612785;
    // 2.793982000179578, 2.9109046618505716, 3.9, 3.0012721635612785;

    g_tilde.resize(3, 4);
    g_tilde << 11.548174312328202, 11.598704156611845, 11.628291546599447, 11.643778865402322,  //
      11.548174312328202, 11.598703702297197, 11.62828695433792, 11.64376300798447,             //
      11.548174312328202, 11.598703247981756, 11.6282823620684, 11.643747150539175;
    // V
    V.resize(3, 4);
    V << 38.26663625588449, 38.59773996101872, 38.796013622121734, 38.87633369598402,  // .......
      38.26663625588449, 38.59773559373169, 38.79595281924629, 38.87610224624988,      //
      38.26663625588449, 38.59773137253549, 38.79589415783469, 38.87588074181628;
    // n
    n.resize(3, 4);
    n << 5.226550889850074, 5.234552609196148, 5.241800129724853, 5.247556359940139,  //
      5.226550889850074, 5.234616581988232, 5.242296468086213, 5.2491802329574115,    // .......
      5.226550889850074, 5.234680554892198, 5.242792807311027, 5.250804108784002;
  }
  std::unique_ptr<tam::common::Track> th;
  Eigen::MatrixXd s;
  Eigen::MatrixXd ax;
  Eigen::MatrixXd ay;
  Eigen::MatrixXd chi;
  Eigen::MatrixXd ax_tilde;
  Eigen::MatrixXd ay_tilde;
  Eigen::MatrixXd g_tilde;
  Eigen::MatrixXd V;
  Eigen::MatrixXd n;
};
TEST_F(acceleration_tests_vegas, calc_a_tilde)
{
  Eigen::MatrixXd mu = th->mu(s);
  Eigen::MatrixXd phi = th->phi(s);

  auto [ax_tilde_res, ay_tilde_res] = tam::common::track::calc_a_tilde(mu, phi, chi, ax, ay);
  ASSERT_EIGEN_MAT(ax_tilde_res, ax_tilde);
  ASSERT_EIGEN_MAT(ay_tilde_res, ay_tilde);
}
TEST_F(acceleration_tests_vegas, calc_g_tilde)
{
  Eigen::MatrixXd mu = th->mu(s);
  Eigen::MatrixXd phi = th->phi(s);
  Eigen::MatrixXd s_dot = tam::common::track::calc_s_dot(V, n, th->omega_z(s), chi);

  auto g_tilde_res =
    tam::common::track::calc_g_tilde(th->omega_x(s), th->omega_y(s), mu, phi, chi, s_dot, V);
  ASSERT_EIGEN_MAT(g_tilde_res, g_tilde);
}
TEST_F(acceleration_tests_vegas, calc_a_hat)
{
  Eigen::MatrixXd mu = th->mu(s);
  Eigen::MatrixXd phi = th->phi(s);

  auto [ax_res, ay_res] = tam::common::track::calc_a_hat(mu, phi, chi, ax_tilde, ay_tilde);
  ASSERT_EIGEN_MAT(ax_res, ax);
  ASSERT_EIGEN_MAT(ay_res, ay);
}
TEST_F(acceleration_tests_vegas, calc_a_tilde_scalar)
{
  for (int i = 0; i < chi.size(); i++) {
    double mu = th->mu(s)(i);
    double phi = th->phi(s)(i);

    auto [ax_tilde_res, ay_tilde_res] =
      tam::common::track::calc_a_tilde(mu, phi, chi(i), ax(i), ay(i));
    ASSERT_FLOAT_EQ(ax_tilde_res, ax_tilde(i));
    ASSERT_FLOAT_EQ(ay_tilde_res, ay_tilde(i));
  }
}
TEST_F(acceleration_tests_vegas, calc_g_tilde_scalar)
{
  for (int i = 0; i < chi.size(); i++) {
    double mu = th->mu(s)(i);
    double phi = th->phi(s)(i);
    double s_dot = tam::common::track::calc_s_dot(V(i), n(i), th->omega_z(s)(i), chi(i));

    auto g_tilde_res = tam::common::track::calc_g_tilde(
      th->omega_x(s)(i), th->omega_y(s)(i), mu, phi, chi(i), s_dot, V(i));
    ASSERT_FLOAT_EQ(g_tilde_res, g_tilde(i));
  }
}
TEST_F(acceleration_tests_vegas, calc_a_hat_scalar)
{
  for (int i = 0; i < chi.size(); i++) {
    double mu = th->mu(s)(i);
    double phi = th->phi(s)(i);

    auto [ax_res, ay_res] =
      tam::common::track::calc_a_hat(mu, phi, chi(i), ax_tilde(i), ay_tilde(i));
    ASSERT_FLOAT_EQ(ax_res, ax(i));
    ASSERT_FLOAT_EQ(ay_res, ay(i));
  }
}
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
