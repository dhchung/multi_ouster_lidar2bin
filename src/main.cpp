#include <iostream>
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/publisher.h"
#include <string>
//Data logger node
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
//Ouster node
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point32.h>

//mkdir
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>

//To write binary files
#include <fstream>

//Threading
#include <thread>

#define PREFIX_PATH "/mnt/DataDisk/"

#include <stdlib.h>

bool data_logging;
std::string data_prefix = "Stoplogging";
std::string stop_logging_msg = "Stoplogging";

std::vector<std::string> dir_lidar_points;

std::vector<std::thread> threads_point;
std::vector<bool> running_check;
int thread_num = 0;
std::vector<std::string> lidar_names;

std::vector<double> last_input_time;

std::vector<FILE*> imu_file;

void dataLoggingFlagCallback(const std_msgs::Bool::ConstPtr &msg){
    if(msg->data) {
        if(data_logging) {

        } else {
            data_logging = true;
            ROS_INFO("Data Logging Set True");
        }
    } else {
        if(data_logging){
            data_logging = false;
            ROS_INFO("Data Logging Set False");

            for(int i = 0; i < threads_point.size(); ++i) {
                if(threads_point[i].joinable()) {
                    threads_point[i].join();
                }
            }

            threads_point.clear();
            running_check.clear();

        }
    }
}

void dataPrefixCallBack(const std_msgs::String::ConstPtr & msg, std::vector<std::string> lidar_names) {
    if(msg->data.compare(stop_logging_msg)!=0) {
        if(data_prefix.compare(stop_logging_msg) == 0) {
            std::cout<<"Prefix changed to logging"<<std::endl;
            data_prefix = msg->data;
            std::string dir_sensor = PREFIX_PATH + data_prefix + "_lidar";
            mkdir(dir_sensor.c_str(), 0777);
            imu_file.resize(lidar_names.size());
            dir_lidar_points.resize(lidar_names.size());

            for(size_t i = 0; i < lidar_names.size(); ++i) {
                std::string dir_lidar_name = dir_sensor + "/" + lidar_names[i];
                mkdir(dir_lidar_name.c_str(), 0777);
                dir_lidar_points[i] = dir_lidar_name + "/points";
                mkdir(dir_lidar_points[i].c_str(), 0777);
                std::string lidar_imu_path = dir_lidar_name + "/imu.txt";
                imu_file[i] = fopen(lidar_imu_path.c_str(), "a");
                if(!imu_file[i])
                    printf("FAILED to open %s\n", lidar_imu_path.c_str()); 
            }

        }
    } else {
        if(data_prefix.compare(stop_logging_msg)!=0) {
            std::cout<<"Prefix changed to stop logging"<<std::endl;
            data_prefix = msg->data;
            for(size_t i = 0; i < lidar_names.size(); ++i) {
                fclose(imu_file[i]);
            }
            imu_file.clear();
            dir_lidar_points.clear();
        }
    }
}

void save_pcd(sensor_msgs::PointCloud2 cloud_t,
              std::string data_dir, 
              int thread_no) {

    std::stringstream ss;
    ss << data_dir << "/" << cloud_t.header.stamp << ".bin";

    std::string data_path = ss.str();

    std::ofstream binfile(data_path.c_str(), std::ios::out | std::ios::binary);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_t, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_t, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_t, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_t, "intensity");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_t(cloud_t, "t");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_reflectivity(cloud_t, "reflectivity");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_ambient(cloud_t, "ambient");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_range(cloud_t, "range");
    // sensor_msgs::PointCloud2Iterator<uint8_t> iter_ring(cloud_t, "ring");

    while(iter_x!=iter_x.end()) {
        binfile.write((char*)&*iter_x, sizeof(float));
        binfile.write((char*)&*iter_y, sizeof(float));
        binfile.write((char*)&*iter_z, sizeof(float));
        binfile.write((char*)&*iter_intensity, sizeof(float));
        binfile.write((char*)&*iter_t, sizeof(uint32_t));
        binfile.write((char*)&*iter_reflectivity, sizeof(uint16_t));
        binfile.write((char*)&*iter_ambient, sizeof(uint16_t));
        binfile.write((char*)&*iter_range, sizeof(uint32_t));
        // binfile.write((char*)&*iter_ring, sizeof(uint8_t));

        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_intensity;
        ++iter_t;
        ++iter_reflectivity;
        ++iter_ambient;
        ++iter_range;
        // ++iter_ring;
    }

    binfile.close();

    running_check[thread_no] = true;


    running_check[thread_no] = false;
}


void point_handle(const sensor_msgs::PointCloud2::ConstPtr & cloud_msg, size_t idx){
    if(data_logging) {
        bool all_thread_running = true;
        int empty_thread = -1;
        for(int i = 0; i < thread_num; ++i) {
            if(running_check[i]){

            } else{
                empty_thread = i;
                all_thread_running = false;
                if(threads_point[i].joinable()){
                    threads_point[i].join();
                }
            }
        }

        sensor_msgs::PointCloud2 cloud_t;
        cloud_t = *cloud_msg;

        if(all_thread_running) {
            running_check.push_back(false);
            threads_point.emplace_back(save_pcd, 
                                       cloud_t, 
                                       dir_lidar_points[idx],
                                       running_check.size()-1);
            threads_point[running_check.size()-1].detach();
            ++thread_num;
        } else {
            threads_point[empty_thread] = std::thread(save_pcd, 
                                                      *cloud_msg,
                                                      dir_lidar_points[idx],
                                                      empty_thread);
            threads_point[empty_thread].detach();    
        }

        int num_running_thread = 0;
        for(int i = 0; i < running_check.size(); ++i) {
            if(running_check[i]) {
                ++num_running_thread;
            }
        }

        std::stringstream ss;
        ss << dir_lidar_points[idx] << "/" << cloud_t.header.stamp << ".bin";

        for(size_t i = 0; i < lidar_names.size(); ++i) {
            std::string blink;
            if(i==idx) {
                blink = "▣";
            } else {
                // blink = "X";
            }

            printf("%s :\t %s\n", lidar_names[i].c_str(), blink.c_str());
        }

        ROS_INFO("[LIDAR] Heard from %s", lidar_names[idx].c_str());
        ROS_INFO("[LIDAR] Data saved to %s", ss.str().c_str());
        ROS_INFO("[LIDAR] Running Thread: %d", num_running_thread);

    } else {
        
    }

}


void imu_handle(const sensor_msgs::Imu::ConstPtr & imu_msg, size_t idx){
    if(data_logging) {
        
        char imu_data_buf[256];

        sprintf(imu_data_buf, "%0.9f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
                               imu_msg->header.stamp.now().toSec(),
                               imu_msg->angular_velocity.x,
                               imu_msg->angular_velocity.y,
                               imu_msg->angular_velocity.z,
                               imu_msg->linear_acceleration.x,
                               imu_msg->linear_acceleration.y,
                               imu_msg->linear_acceleration.z,
                               imu_msg->orientation.w,
                               imu_msg->orientation.x,
                               imu_msg->orientation.y,
                               imu_msg->orientation.z);
        fwrite(imu_data_buf, 1, strlen(imu_data_buf), imu_file[idx]);
    } else {
        
    }

}

int main(int argc, char** argv) {

    if(argc == 1) {
        std::cout<<"No typed lidar names"<<std::endl;
        return 0;
    }
    lidar_names.clear();

    for(int i = 0; i <argc-1; ++i) {
        lidar_names.push_back(std::string(argv[i+1]));
        std::cout<<"lidar_names: "<<lidar_names[i]<<std::endl;
    }

    ros::init(argc, argv, "save_ptimu");
    std::cout<<"[POINTCLOUD SAVE NODE]"<<std::endl;
    std::cout<<"Data save path set to:"<<PREFIX_PATH<<std::endl;

    ros::NodeHandle nh;
    ros::Subscriber sub_bool = nh.subscribe("/datalogging", 1, dataLoggingFlagCallback);
    ros::Subscriber sub_prefix = nh.subscribe<std_msgs::String>("/save_prefix", 1, boost::bind(&dataPrefixCallBack, _1, lidar_names));

    std::vector<ros::Subscriber> sub_points(lidar_names.size());
    std::vector<ros::Subscriber> sub_imus(lidar_names.size());

    for(size_t i = 0; i<lidar_names.size(); ++i) {
        std::string lidar_topic_name = "/" + lidar_names[i] + "/os_cloud_node/points";
        std::string lidar_imu_topic_name = "/" + lidar_names[i] + "/os_cloud_node/imu";
        sub_points[i] = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic_name, 1, boost::bind(&point_handle, _1, i));
        sub_imus[i] = nh.subscribe<sensor_msgs::Imu>(lidar_imu_topic_name, 1, boost::bind(&imu_handle, _1, i));
        std::cout<<"Subscribing to /"<<lidar_names[i]<<"/os_cloud_node/points"<<std::endl;
        std::cout<<"Subscribing to /"<<lidar_names[i]<<"/os_cloud_node/imu"<<std::endl;
    }

    while(ros::ok()) {
        ros::spinOnce();
    }
    // ros::spin();
 
    for(int i = 0; i<threads_point.size(); ++i) {
        threads_point[i].join();
    }

    return 0;
}