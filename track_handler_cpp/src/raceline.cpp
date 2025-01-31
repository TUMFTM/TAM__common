// Copyright 2023 Simon Hoffmann
#include "track_handler_cpp/raceline.hpp"
#include "track_handler_cpp/track_io.hpp"

#include <memory>
#include <string>

#include "tum_helpers_cpp/file_handling.hpp"
namespace tam::common
{
Raceline::Raceline(RacelineData && data)
{
  data_ = std::make_unique<RacelineData>(std::move(data));
  if (!tam::helpers::track::has_equal_sized_data(data_->data)){
      throw std::invalid_argument(
        "[track_handler_cpp]: Trying to load raceline content with unequal size");
  }
}
std::unique_ptr<Raceline> Raceline::create_from_csv(const std::string & path)
{  // make unique doesn't allow protected access
  return std::unique_ptr<Raceline>(new Raceline(tam::common::get_raceline_from_file(path)));
}
}  // namespace tam::common
