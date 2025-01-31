// Copyright 2023 Simon Hoffmann
#pragma once
#include <algorithm>
namespace tam::helpers::utility
{
/**
 * \brief (Bounded) Index Counter
 *
 * Starts at 0 and stops at max_val. If is_circular starts over at 0 if max_val is reached
 */
class IndexCounter
{
public:
  explicit IndexCounter(int max_val, bool is_circular = false)
  : max_val_{max_val}, is_circular_{is_circular}
  {
  }
  IndexCounter & operator++()
  {
    set(index_ + 1);
    return *this;
  }
  IndexCounter operator++(int)
  {
    IndexCounter temp = *this;
    set(index_ + 1);
    return temp;
  }
  IndexCounter & operator--()
  {
    set(index_ - 1);
    return *this;
  }
  IndexCounter operator--(int)
  {
    IndexCounter temp = *this;
    set(index_ - 1);
    return temp;
  }
  int operator()() const { return index_; }
  int operator()(const int & offset) const { return calc_index(index_ + offset); }
  void set(int number) { index_ = calc_index(number); }

private:
  int calc_index(const int & input) const
  {
    if (!is_circular_) {
      return std::clamp(input, 0, max_val_);
    } else {
      if (input < 0) {
        return max_val_ - (std::abs(input) % (max_val_ + 1)) + 1;
      } else {
        return (input % (max_val_ + 1));
      }
    }
  }
  int max_val_{0};
  int index_{0};
  bool is_circular_{false};
};
}  // namespace tam::helpers::utility
