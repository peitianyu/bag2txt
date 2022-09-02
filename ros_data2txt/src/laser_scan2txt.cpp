#include <fstream>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <signal.h>
#include <tf/tf.h>

#include <sensor_msgs/LaserScan.h>

using namespace std;

static std::ofstream out_file;


void MySigintHandler(int sig)
{
    //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
    ROS_INFO("laser_scan2txt shutting down!");
    ros::shutdown();
    out_file.close();
}

void SubLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    static uint last_seq = 0;

    if(msg->header.seq != last_seq)
    {
        uint64_t time_stamp = msg->header.stamp.sec * 1e3 + msg->header.stamp.nsec / 1e6 - 165874 * 1e7;
        out_file << time_stamp << " " << msg->ranges.size() << " " << msg->angle_min << " " << msg->angle_max << " " << 
                msg->angle_increment << " " << msg->range_min << " " << msg->range_max << " "<< msg->time_increment * 1e3 * msg->ranges.size()<< " ";

        for(int i = 0; i < msg->ranges.size(); ++i)
            {out_file << msg->ranges[i] << " ";}
        for(int i = 0; i < msg->intensities.size(); ++i)
            {out_file << msg->intensities[i] << " ";}
        out_file << endl;
    }
    last_seq = msg->header.seq;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_scan2txt");
    ros::NodeHandle nh("~");

    signal(SIGINT, MySigintHandler);

    std::string file_name;
    std::string topic_name;
    nh.param<std::string>("file_name", file_name, "./laser_scan_data.txt");
    nh.param<std::string>("topic_name", topic_name, "/scan");

    out_file.open(file_name, ios::out);
    if(!out_file.is_open()){
        cout << "open file failed!" << endl;
        return -1;
    }

    ros::Subscriber sub_scan = nh.subscribe(topic_name, 100, SubLaserScanCallback);

    ros::spin();
    return 0;
}