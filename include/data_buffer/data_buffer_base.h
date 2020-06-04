#ifndef DATA_BUFFER_DATA_BUFFER_BASE_H_INCLUDED
#define DATA_BUFFER_DATA_BUFFER_BASE_H_INCLUDED

//headers in STL
#include <vector>
#include <map>
#include <mutex>
#include <cmath>

//headers in ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace data_buffer
{
template<class T>
class DataBufferBase
{
public:
  DataBufferBase(rclcpp::Clock::SharedPtr clock, std::string key, double buffer_length)
  : key(key), buffer_length(buffer_length)
  {
    ros_clock_ = clock;
    data_ = std::vector<T>(0);
  }
  ~DataBufferBase() {}
  void queryData(rclcpp::Time from, rclcpp::Time to, std::vector<T> & ret)
  {
    update();
    mtx.lock();
    ret.clear();
    if (data_.size() == 0) {
      mtx.unlock();
      return;
    }
    for (auto itr = data_.begin(); itr != data_.end(); itr++) {
      try {
        if (itr->header.stamp > from && itr->header.stamp < to) {
          ret.push_back(*itr);
        }
      } catch (std::exception & e) {
        mtx.unlock();
        return;
      }
    }
    mtx.unlock();
  }
  void addData(T data)
  {
    update();
    mtx.lock();
    data_.push_back(data);
    mtx.unlock();
  }
  bool queryData(rclcpp::Time timestamp, T & data)
  {
    mtx.lock();
    try {
      data = T();
      std::vector<T> data_array = getData();
      if (data_array.size() == 0) {
        mtx.unlock();
        return false;
      }
      if (data_array.size() == 1) {
        data = data_array[0];
        mtx.unlock();
        return true;
      }
      rclcpp::Time head_stmap = data_array[0].header.stamp;
      if (head_stmap > timestamp) {
        mtx.unlock();
        return false;
      }
      rclcpp::Time end_stmap = data_array[data_array.size() - 1].header.stamp;
      if (end_stmap < timestamp) {
        int index = data_array.size() - 2;
        data = interpolate(data_array[index], (data_array)[index + 1], timestamp);
        mtx.unlock();
        return true;
      }
      for (int i = 0; i < static_cast<int>(data_array.size() - 1); i++) {
        rclcpp::Time t0_stmap = data_array[i].header.stamp;
        rclcpp::Time t1_stmap = data_array[i + 1].header.stamp;
        if (t0_stmap < timestamp && timestamp < t1_stmap) {
          data = interpolate(data_array[i], data_array[i + 1], timestamp);
          mtx.unlock();
          return true;
        }
      }
    } catch (std::exception & e) {
      mtx.unlock();
      return false;
    }
    mtx.unlock();
    return false;
  }
  virtual T interpolate(T data0, T data1, rclcpp::Time stamp) = 0;
  const std::string key;
  const double buffer_length;
  std::mutex mtx;
  std::vector<T> getData() {return data_;}

protected:
  double toSec(rclcpp::Duration duration)
  {
    double ret;
    double nsecs = static_cast<double>(duration.nanoseconds());
    ret = nsecs * std::pow((double)10.0, -9);
    return ret;
  }

private:
  rclcpp::Clock::SharedPtr ros_clock_;
  std::vector<T> data_;
  bool compareTimeStamp(T data0, T data1)
  {
    return data0.header.stamp < data1.header.stamp;
  }
  void reorderData()
  {
    mtx.lock();
    std::sort(data_.begin(), data_.end(),
      std::bind(&DataBufferBase::compareTwistTimeStamp, this, std::placeholders::_1,
      std::placeholders::_2));
    mtx.unlock();
  }
  void update()
  {
    std::vector<T> data;
    mtx.lock();
    rclcpp::Time now = ros_clock_->now();
    rclcpp::Time target_timestamp = now - rclcpp::Duration(buffer_length);
    for (auto itr = data_.begin(); itr != data_.end(); itr++) {
      rclcpp::Time header_stamp = itr->header.stamp;
      if (header_stamp > target_timestamp) {
        data.push_back(*itr);
      }
    }
    data_ = data;
    mtx.unlock();
  }
};
}
#endif  //DATA_BUFFER_DATA_BUFFER_BASE_H_INCLUDED
