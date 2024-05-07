#include <parallel_filters.h>
#include <iostream>
#include <state.hpp>
using namespace filters;

Filter::Filter() {}
Filter::~Filter() { delete integrator_; }
void Filter::StateSynchronize(const Filter* const filter) {
  filter_state_ = filter->filter_state_;
}
void Filter::Initialize(State state) { filter_state_ = state; }
ParallelFilterA::ParallelFilterA() { eskf_ = new ErrorStateKalmanFilter(); }
void ParallelFilterA::Initialize(const State state) {
  filter_state_ = state;
  eskf_->Initialize();
}
ParallelFilterA::~ParallelFilterA() { delete eskf_; }