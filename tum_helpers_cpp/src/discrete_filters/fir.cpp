// Copyright 2023 Marcel Weinmann
#include "tum_helpers_cpp/discrete_filters/fir.hpp"

tam::core::state::FIR::FIR(const Eigen::Ref<const Eigen::VectorXd> & coefficients)
{
    // initialize the filter coefficients
    filter_coefficients_ = coefficients;
    fifo_.resize(coefficients.size(), 0);
}

/**
 * @brief Step the FIR filter once
 *
 * @param[in] input         - double:
 *                            value to be inserted in the buffer
 * @param[out]              - double:
 *                            filtered value by the FIR filter
 */
double tam::core::state::FIR::step(double input)
{
    double output = 0.0;

    // update the FIFO of the FIR buffer
    fifo_.push_front(input);
    fifo_.pop_back();

    // calculate the dot product between FIR and coefficients
    for (int i=0; i < filter_coefficients_.size(); i++) {
        output += filter_coefficients_[i] * fifo_[i];
    }

    return output;
}
