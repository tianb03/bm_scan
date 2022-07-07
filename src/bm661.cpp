#include <ros/ros.h> 
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <sched.h>

#include <sys/select.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <math.h>
#include "../include/data_type.h"

#pragma pack(push,1)
#define DEG2RAD(x) ((x)*M_PI / 180.f)

#define __autoalign__ __attribute__((packed))

typedef struct
{
    unsigned short dist1;
    unsigned char  rssi1;
	unsigned short dist2;
    unsigned char  rssi2;
} __autoalign__ channel;

typedef struct
{
    unsigned short flag;
    unsigned short azimuth;
    channel ch[16];
}__autoalign__ data_block;

struct
{
    data_block block[12];
    unsigned int timestamp;
    unsigned short factory;
}__autoalign__ data_package;

#pragma pack(pop)

std::vector <bm_response_scan_t> scan_vec;
int resolution = 25;
int scan_vec_ready = 0;
int angle_offset = 0;

std::string hostip, port, frame_id, scan_topic;
bool inverted;

ros::Time scan_begin, scan_end;

int main(int argc, char **argv) 
{
	pthread_t publish_t, recv_t;
	struct sockaddr_in ser_addr, clent_addr; 
	int i = 0, j = 12;
	int sockfd;

	ros::init(argc, argv, "laser_scan_publisher");
	ros::NodeHandle nh;
	ros::Rate rate(30); 

	ros::param::get("/bm661/frame_id", frame_id);
	ros::param::get("/bm661/scan_topic", scan_topic);
  	ros::param::get("/bm661/inverted", inverted);
	ros::param::get("/bm661/hostip", hostip);
	ros::param::get("/bm661/port", port);
	ros::param::get("/bm661/angle_offset", angle_offset);

	ROS_INFO("frame_id:%s", frame_id.c_str());
	ROS_INFO("scan_topic:%s", scan_topic.c_str());
	ROS_INFO("inverted:%s", (inverted ? "True" : "False"));
	ROS_INFO("hostip:%s", hostip.c_str());
	ROS_INFO("port:%s", port.c_str());

	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan> (scan_topic, 1000);

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd == -1)
	{
		ROS_INFO("Failed to create socket");
		return -1;
    }

	memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
	ser_addr.sin_addr.s_addr = inet_addr(hostip.c_str());
    ser_addr.sin_port = htons(atoi(port.c_str()));

    if(bind(sockfd, (struct sockaddr*)&ser_addr, sizeof(ser_addr)) < 0)
	{
		ROS_INFO("Socket bind error!");
		return -1;
	}

	while (ros::ok())
	{
		if(scan_vec_ready == 0)
		{
			while(1)
			{
				if(j == 12)
				{
					scan_begin = ros::Time::now();
					unsigned int len = sizeof(clent_addr);
					recvfrom(sockfd, &data_package, sizeof(data_package), 0, (struct sockaddr*)&clent_addr, &len);
					j = 0;
				}

				if((data_package.block[1].azimuth - data_package.block[0].azimuth) > 0)
				{
					resolution = (data_package.block[1].azimuth - data_package.block[0].azimuth) / 16;
				}

				for(;j < 12; j++)
				{
					for(i = 0; i < 16; i++)
					{
						bm_response_scan_t response_ptr;
						response_ptr.angle = (data_package.block[j].azimuth + (resolution * i));

						if(response_ptr.angle == 0)
						{
							if(!scan_vec.empty() & (scan_vec_ready == 0))
							{
								scan_vec_ready = 1;
								if(scan_vec.size() < 1200)
								{
									j = 12;
								}
								break;
							}
						}
						response_ptr.dist1 = data_package.block[j].ch[i].dist1;
						response_ptr.dist2 = data_package.block[j].ch[i].dist2;
						response_ptr.rssi1 = data_package.block[j].ch[i].rssi1;
						response_ptr.rssi2 = data_package.block[j].ch[i].rssi2;
						scan_vec.push_back(response_ptr);
					}
					if(scan_vec_ready == 1)
					{
						break;
					}
				}
				if(scan_vec_ready == 1)
				{
					break;
				}
			}
		}

		if(scan_vec_ready == 1)
		{
			scan_end = ros::Time::now();

			sensor_msgs::LaserScan scan;
			uint16_t num_readings;
			float duration = (scan_end - scan_begin).toSec();
			
			num_readings = scan_vec.size();
			scan.header.stamp = scan_begin;
			scan.header.frame_id = frame_id;
			scan.angle_min = DEG2RAD(-180 + angle_offset);
			scan.angle_max = DEG2RAD(180 + angle_offset);
			scan.angle_increment = 2.0 * M_PI / num_readings;
			scan.scan_time = duration;
			scan.time_increment = duration/(float)num_readings;
			scan.range_min = 0.0;
			scan.range_max = 30.0;
			scan.ranges.resize(num_readings);
			scan.intensities.resize(num_readings);

			for(int i = 0;i < num_readings; i++)
			{
				if (!inverted) 
    			{
					scan.ranges[i] = (float)scan_vec[i].dist1 / 1000;
					scan.intensities[i] = scan_vec[i].rssi1;
				}
				else
				{
					scan.ranges[num_readings - i - 1] = (float)scan_vec[i].dist1 / 1000;
					scan.intensities[num_readings - i - 1] = scan_vec[i].rssi1;
				}
			}

			scan_pub.publish(scan);
			ROS_DEBUG("New scan published, total data points: %d", num_readings);
			scan_vec.clear();
			scan_vec_ready = 0;
		}
	}
	close(sockfd);

	return 0;
}
