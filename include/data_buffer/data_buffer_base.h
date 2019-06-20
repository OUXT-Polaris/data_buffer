#ifndef DATA_BUFFER_DATA_BUFFER_BASE_H_INCLUDED
#define DATA_BUFFER_DATA_BUFFER_BASE_H_INCLUDED

//headers in STL
#include <vector>
#include <map>
#include <mutex>

//headers in ROS
#include <ros/ros.h>

namespace data_buffer
{
    template<class T>
    class DataBufferBase
    {
    public:
        DataBufferBase(std::string key,double buffer_length) : key(key),buffer_length(buffer_length)
        {
            data_ = std::vector<T>(0);
        }
        ~DataBufferBase(){}
        void queryData(ros::Time from,ros::Time to,std::vector<T>& ret)
        {
            update();
            mtx.lock();
            ret.clear();
            if(data_.size() == 0)
            {
                mtx.unlock();
                return;
            }
            for(auto itr = data_.begin(); itr != data_.end(); itr++)
            {
                try
                {
                    if(itr->header.stamp > from && itr->header.stamp < to)
                    {
                        ret.push_back(*itr);
                    }
                }
                catch(std::exception& e)
                {
                    ROS_ERROR_STREAM(e.what());
                    mtx.unlock();
                    return;
                }
            }
            mtx.unlock();
            return;
        }
        void addData(T data)
        {
            update();
            mtx.lock();
            data_.push_back(data);
            mtx.unlock();
            return;
        }
        bool queryData(ros::Time timestamp, T& data)
        {
            mtx.lock();
            try
            {
                data = T();
                std::vector<T> data_array = getData();
                if(data_array.size() == 0)
                {
                    mtx.unlock();
                    return false;
                }
                if(data_array.size() == 1)
                {
                    data = data_array[0];
                    mtx.unlock();
                    return true;
                }
                if(data_array[0].header.stamp > timestamp)
                {
                    mtx.unlock();
                    return false;
                }
                if(data_array[data_array.size()-1].header.stamp < timestamp)
                {
                    int index = data_array.size()-2;
                    data = interpolate(data_array[index],(data_array)[index+1],timestamp);
                    mtx.unlock();
                    return true;
                }
                for(int i =0; i<data_array.size()-1; i++)
                {
                    if(data_array[i].header.stamp < timestamp && timestamp < data_array[i+1].header.stamp)
                    {
                        data = interpolate(data_array[i],data_array[i+1],timestamp);
                        mtx.unlock();
                        return true;
                    }
                }
            }
            catch(std::exception& e)
            {
                ROS_ERROR_STREAM(e.what());
                mtx.unlock();
                return false;
            }
            mtx.unlock();
            return false;
        }
        virtual T interpolate(T data0,T data1,ros::Time stamp) = 0;
        const std::string key;
        const double buffer_length;
        std::mutex mtx;
        std::vector<T> getData(){return data_;};
    private:
        std::vector<T> data_;
        bool compareTimeStamp(T data0,T data1)
        {
            return data0.header.stamp < data1.header.stamp;
        }
        void reorderData()
        {
            mtx.lock();
            std::sort(data_.begin(), data_.end(), std::bind(&DataBufferBase::compareTwistTimeStamp, this,std::placeholders::_1, std::placeholders::_2));
            mtx.unlock();
        }
        void update()
        {
            std::vector<T> data;
            mtx.lock();
            ros::Time now = ros::Time::now();
            ros::Time target_timestamp = now - ros::Duration(buffer_length);
            for(auto itr = data_.begin(); itr != data_.end(); itr++)
            {
                if(itr->header.stamp > target_timestamp)
                {
                    data.push_back(*itr);
                }
            }
            data_ = data;
            mtx.unlock();
        }
    };
}
#endif  //DATA_BUFFER_DATA_BUFFER_BASE_H_INCLUDED