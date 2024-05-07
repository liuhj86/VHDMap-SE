#ifndef VHDMAPSE_INCLUDE_PARALLEL_FILTERS_H_
#define VHDMAPSE_INCLUDE_PARALLEL_FILTERS_H_

#include <error_state_kalman_filter.h>
#include <integrator.hpp>
#include <state.hpp>

namespace filters {
class Filter {
 public:
  Filter();
  virtual ~Filter();
  Integrator* integrator_;
  State filter_state_;
  virtual void Initialize(const State state);
  void StateSynchronize(const Filter* const filter);
};
class ParallelFilterA : public Filter {  // filter 1
 public:
  ParallelFilterA();
  ~ParallelFilterA();
  void Initialize(const State state);
  ErrorStateKalmanFilter* eskf_;
};
class ParallelFilterB : public Filter {
 public:
  ParallelFilterB() {}
  ~ParallelFilterB() {}
};
}  // namespace filters
#endif  // VHDMAPSE_INCLUDE_PARALLEL_FILTERS_H_
