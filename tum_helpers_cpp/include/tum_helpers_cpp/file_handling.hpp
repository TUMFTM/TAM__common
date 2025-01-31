// Copyright 2023 Simon Hoffmann
#pragma once
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
namespace tam::helpers::files
{
template <typename M>
static M load_csv(
  const std::string & path, const int header_lines = 1, const char comment_char = '#')
{
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("[tam::helpers::files::load_csv]: File does not exist " + path);
  }
  std::ifstream indata;
  indata.open(path);
  std::string line;
  std::vector<double> values;
  uint rows = 0;
  uint n = 1;
  while (std::getline(indata, line)) {
    if (line.at(0) == comment_char) continue;
    if (n <= header_lines) {
      ++n;
      continue;
    }
    std::stringstream lineStream(line);
    std::string cell;
    while (std::getline(lineStream, cell, ',')) {
      if (cell.empty()){
        values.push_back(std::nan("1"));
      } else {
        values.push_back(std::stod(cell));
      }
    }
    ++n;
    ++rows;
  }
  return Eigen::Map<const Eigen::Matrix<
    typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(
    values.data(), rows, values.size() / rows);
}
static std::vector<std::string> get_csv_header(
  const std::string & path, const int header_lines = 1, const char comment_char = '#')
{
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("[tam::helpers::files::load_csv]: File does not exist " + path);
  }
  std::ifstream indata;
  indata.open(path);
  std::string line;
  std::vector<std::string> values;
  uint rows = 0;
  uint n = 1;
  while (std::getline(indata, line)) {
    if (line.at(0) == comment_char) continue;
    if (n == header_lines) {
      std::stringstream lineStream(line);
      std::string cell;
      while (std::getline(lineStream, cell, ',')) {
        values.push_back(cell);
      }
      return values;
    }
    ++n;
    ++rows;
  }
}
}  // namespace tam::helpers::files
