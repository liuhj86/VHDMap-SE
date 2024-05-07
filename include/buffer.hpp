#ifndef VHDMAPSE_INCLUDE_BUFFER_HPP_
#define VHDMAPSE_INCLUDE_BUFFER_HPP_

#include <ros/ros.h>
#include <iostream>
#include <map>
#include <utility>
#include <vector>

template <typename MeasurementData>
class MapRingBuffer {
 public:
  std::map<double, MeasurementData> measurement_map_;
  typename std::map<double, MeasurementData>::iterator measurement_it_;
  typename std::map<double, MeasurementData>::reverse_iterator
      measurement_it_reverse_;

  int size_;
  double max_wait_time_;
  double min_wait_time_;

  MapRingBuffer() {
    max_wait_time_ = 0.1;
    min_wait_time_ = 0.0;
  }

  virtual ~MapRingBuffer() {}

  bool Allocate(const int buffer_size) {
    if (buffer_size <= 0) {
      return false;
    } else {
      size_ = buffer_size;
      return true;
    }
  }

  int GetSize() { return measurement_map_.size(); }

  void AddMeasurementData(const MeasurementData& measurement_data, double t) {
    bool insert_result =
        measurement_map_.insert(std::make_pair(t, measurement_data)).second;
    if (measurement_map_.size() > size_) {
      measurement_map_.erase(measurement_map_.begin());
    }
  }

  void CleanMeasurementBefore(
      double t) {  // Clear the measured values before time t
    while (measurement_map_.size() >= 1 &&
           measurement_map_.begin()->first <= t) {
      measurement_map_.erase(measurement_map_.begin());
    }
  }

  bool GetLastTime(double* last_time) {
    if (!measurement_map_.empty()) {
      (*last_time) = measurement_map_.rbegin()->first;
      return true;
    } else {
      return false;
    }
  }

  bool GetLastMeasurementData(MeasurementData* last_measurement) {
    if (!measurement_map_.empty()) {
      (*last_measurement) = measurement_map_.rbegin()->second;
      return true;
    } else {
      return false;
    }
  }

  bool GetPenultimateMeasurementData(
      MeasurementData*
          penultimate_measurement) {  // Get the second to last data
    if (measurement_map_.size() >= 2) {
      measurement_it_reverse_ = measurement_map_.rbegin();
      measurement_it_reverse_++;
      (*penultimate_measurement) = measurement_it_reverse_->second;
      return true;
    } else {
      // std::cout<<measurement_map_.size()<<std::endl;
      return false;
    }
  }

  bool GetFirstMeasurementData(MeasurementData* first_measurement) {
    if (!measurement_map_.empty()) {
      (*first_measurement) = measurement_map_.begin()->second;
      return true;
    } else {
      return false;
    }
  }

  bool Empty() { return measurement_map_.empty(); }
};

#endif  // VHDMAPSE_INCLUDE_BUFFER_HPP_
