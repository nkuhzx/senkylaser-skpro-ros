// Copyright [nkuhzx] [name of copyright owner]

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <senkylaser.h>
#include <sensor_msgs/LaserScan.h>
#include <unistd.h>


using namespace std;


class SklaserFactory
{
public:
    SklaserFactory();

    ~SklaserFactory(){};

    void init();

    void get_Device();

    void publish_Topic();



private:

    ros::NodeHandle sklaser_node;

    ros::Publisher sklaser_pub;
    
    SenkyLaser sklaser;



};

SklaserFactory::SklaserFactory()
{
    ROS_INFO("Senkylaser ROS");

    string port_name;
    int service_id;

    sklaser_node=ros::NodeHandle("~");

    sklaser_node.getParam("port",port_name);
    sklaser_node.getParam("service_id",service_id);

    sklaser=SenkyLaser(port_name,service_id);
}


void SklaserFactory::init()
{
    this->sklaser_pub=this->sklaser_node.advertise<sensor_msgs::LaserScan>("sklaser",1);

    int flag=0;
    int num=0;
    char c;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        flag=sklaser.openSerial();
        // flag=-1;
        if(flag==0)
        {
            ROS_ERROR("[Senkylaser] wrong modbus parameters");
        }
        else if(flag==-1)
        {
            ROS_ERROR_STREAM("[Senkylaser] Cannot connect modubs at port "<<sklaser.getPort());
        }
        else if(flag==1)
        {
            ROS_INFO_STREAM("[Senkylaser] Connect modubs at port "<<sklaser.getPort());
            break;
        }

        loop_rate.sleep();
    }


}

void SklaserFactory::get_Device()
{
    this->sklaser.SetLaserDeivce();



    ros::Rate loop_rate(60);
    while(ros::ok())
    {
        int flag=this->sklaser.GetLaserDeviceAddress();
        if(flag==-1)
        {
            ROS_ERROR("[Senkylaser] Connected Error");           
        }
        else
        {
            ROS_INFO("[Senkylaser] Connected to Senkylaser !!");

            sklaser.GetLaserSerialNum();

            ROS_INFO_STREAM("[Senkylaser] Device with serial number "<<sklaser.getSerialnum());

            break;
        }
        loop_rate.sleep();
    }

    


}


void SklaserFactory::publish_Topic()
{
    sklaser.SetLaserFreq(FREQ_30HZ);
    int distance;
    int errorstate;

    ros::Rate loop_rate(40);

    while(ros::ok())
    {   
        errorstate=sklaser.GetLaserErrorState();

        distance=sklaser.GetLaserDis();

        ros::Time measure_time=ros::Time::now();
        sensor_msgs::LaserScan laser_msg;

        laser_msg.header.stamp=measure_time;
        laser_msg.header.frame_id="laser_frame";
        laser_msg.angle_min=0;
        laser_msg.angle_max=0;
        laser_msg.angle_increment=0;

        laser_msg.range_min=0.05;
        laser_msg.range_max=30;

        laser_msg.ranges.push_back(distance/10000.0);
        laser_msg.intensities.push_back(errorstate);

        this->sklaser_pub.publish(laser_msg);

        loop_rate.sleep();
    }

}


int main(int argc,char **argv)
{

    ros::init(argc,argv,"skpro30_laser");

    SklaserFactory sktest;   

    sktest.init(); 

    sktest.get_Device();

    sktest.publish_Topic();

    ros::spin();
}