// Copyright 2024 Maximilian Leitenstern
#pragma once

#include "tum_types_cpp/common.hpp"

namespace tam::types::tire_models
{
struct TireModelOutput
{
  tam::types::common::Vector2D<double> force{0.0, 0.0};  // Forces in tire frame
  double self_aligning_moment{0.0};                      // Self aligning moment
};
struct MF_simple
{
  struct Coefficients
  {
    double B{0.0};
    double C{0.0};
    double D{0.0};
    double E{0.0};
  };
  Coefficients lon, lat;
};
struct MF_52
{
  double FNOMIN{0.0};
  double LCX{0.0};
  double LCY{0.0};
  double LEX{0.0};
  double LEY{0.0};
  double LFZO{0.0};
  double LGAX{0.0};
  double LGAY{0.0};
  double LGAZ{0.0};
  double LGYR{0.0};
  double LHX{0.0};
  double LHY{0.0};
  double LKX{0.0};
  double LKY{0.0};
  double LKYG{0.0};
  double LMUX{0.0};
  double LMUY{0.0};
  double LMX{0.0};
  double LMY{0.0};
  double LRES{0.0};
  double LS{0.0};
  double LSGAL{0.0};
  double LSGKP{0.0};
  double LTR{0.0};
  double LVMX{0.0};
  double LVX{0.0};
  double LVY{0.0};
  double LVYKA{0.0};
  double LXAL{0.0};
  double LYKA{0.0};
  double PCX1{0.0};
  double PCY1{0.0};
  double PDX1{0.0};
  double PDX2{0.0};
  double PDX3{0.0};
  double PDY1{0.0};
  double PDY2{0.0};
  double PDY3{0.0};
  double PEX1{0.0};
  double PEX2{0.0};
  double PEX3{0.0};
  double PEX4{0.0};
  double PEY1{0.0};
  double PEY2{0.0};
  double PEY3{0.0};
  double PEY4{0.0};
  double PEY5{0.0};
  double PHX1{0.0};
  double PHX2{0.0};
  double PHY1{0.0};
  double PHY2{0.0};
  double PHY3{0.0};
  double PKX1{0.0};
  double PKX2{0.0};
  double PKX3{0.0};
  double PKY1{0.0};
  double PKY2{0.0};
  double PKY3{0.0};
  double PKY4{0.0};
  double PKY5{0.0};
  double PKY6{0.0};
  double PKY7{0.0};
  double PPY1{0.0};
  double PPY2{0.0};
  double PPY3{0.0};
  double PPY4{0.0};
  double PPY5{0.0};
  double PTX1{0.0};
  double PTX2{0.0};
  double PTX3{0.0};
  double PTY1{0.0};
  double PTY2{0.0};
  double PVX1{0.0};
  double PVX2{0.0};
  double PVY1{0.0};
  double PVY2{0.0};
  double PVY3{0.0};
  double PVY4{0.0};
  double RBX1{0.0};
  double RBX2{0.0};
  double RBX3{0.0};
  double RBY1{0.0};
  double RBY2{0.0};
  double RBY3{0.0};
  double RBY4{0.0};
  double RCX1{0.0};
  double RCY1{0.0};
  double REX1{0.0};
  double REX2{0.0};
  double REY1{0.0};
  double REY2{0.0};
  double RHX1{0.0};
  double RHY1{0.0};
  double RHY2{0.0};
  double RVY1{0.0};
  double RVY2{0.0};
  double RVY3{0.0};
  double RVY4{0.0};
  double RVY5{0.0};
  double RVY6{0.0};
};
}  // namespace tam::types::tire_params
