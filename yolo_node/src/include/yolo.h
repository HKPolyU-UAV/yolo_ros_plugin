/*
    This file is part of YOLO_ROS_PLUGIN - the non-robocentric dynamic landing system for quadrotor

    YOLO_ROS_PLUGIN is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    YOLO_ROS_PLUGIN is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with YOLO_ROS_PLUGIN.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file yolo.h
 * \date 01/07/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief //legacy of AUTO (https://github.com/HKPolyU-UAV/AUTO) & previous 1st gen YOLO_ROS_PLUGIN (https://www.mdpi.com/1424-8220/22/1/404)
 */

#ifndef YOLO_H
#define YOLO_H

#include "essential.h"

#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <opencv2/dnn.hpp>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pthread.h>


#include "RosTopicConfigs.h"
// map definition for convinience

#define COLOR_RAW_SUB_TOPIC CAMERA_SUB_TOPIC_A
#define COLOR_COMP_SUB_TOPIC CAMERA_SUB_TOPIC_B

#define DEPTH_RAW_SUB_TOPIC CAMERA_SUB_TOPIC_C
#define DEPTH_COMP_SUB_TOPIC CAMERA_SUB_TOPIC_D

#define IMAGE_PUB_TOPIC IMAGE_PUB_TOPIC_A

namespace mission_control_center{

    typedef struct objectinfo
    {
        float confidence = 0;
        std::string classnameofdetection;
        cv::Rect boundingbox;
        cv::Mat frame;
        double depth;
    }objectinfo;

    class CnnNodelet : public nodelet::Nodelet
    {
        private:

            cv::Mat frame;
            bool initiated = false;

            

            message_filters::Subscriber<sensor_msgs::Image> subimage_raw;
            message_filters::Subscriber<sensor_msgs::Image> subdepth_raw;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy_RAW;
            typedef message_filters::Synchronizer<MySyncPolicy_RAW> sync_raw;
            boost::shared_ptr<sync_raw> sync_raw_;

            message_filters::Subscriber<sensor_msgs::CompressedImage> subimage_comp;
            message_filters::Subscriber<sensor_msgs::Image> subdepth_comp;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy_COMP;
            typedef message_filters::Synchronizer<MySyncPolicy_COMP> sync_comp;
            boost::shared_ptr<sync_comp> sync_comp_;

            ros::Subscriber rgb_sub;

            cv::String weightpath; 
            cv::String cfgpath; 
            cv::String classnamepath;

            cv::Mat depthdata;

            float set_confidence;
            std::vector<std::string> classnames;

            void findboundingboxes(cv::Mat &frame);
            void findwhichboundingboxrocks(std::vector<cv::Mat> &netOut, cv::Mat &frame);
            void getclassname(std::vector<std::string> &classnames);
            std::chrono::time_point <std::chrono::steady_clock> total_start, total_end, dnn_start, dnn_end;
            float total_fps;
            objectinfo obj;
            cv::dnn::Net mydnn;
            
            void CnnNodeletInitiate(const cv::String cfgfile, const cv::String weightfile, const cv::String classnamepath);


            void rundarknet(cv::Mat &frame);
            void display(cv::Mat frame);
            void getdepthdata(cv::Mat depthdata);
            float appro_fps;
            std::vector<objectinfo> obj_vector;


            void color_image_raw_callback(
                const sensor_msgs::ImageConstPtr &rgbmsg
            );

            void color_image_compressed_callback(
                const sensor_msgs::CompressedImageConstPtr &rgbmsg
            );

            void color_depth_image_raw_callback(
                const sensor_msgs::ImageConstPtr & rgbmsg, 
                const sensor_msgs::ImageConstPtr & depthmsg
            );

            void color_depth_image_compressed_callback(
                const sensor_msgs::CompressedImageConstPtr & rgbmsg, 
                const sensor_msgs::ImageConstPtr & depthmsg
            );

            void set_image_to_publish();

            image_transport::Publisher pubimage;
            
            int input_type = 100;
            double CNN_size = 608;
            bool w_GPU = false;

            
            virtual void onInit() 
            {
                ros::NodeHandle& nh = getNodeHandle();

                RosTopicConfigs configs(nh, "/cnn");

                nh.getParam("/cnn/weightpath", weightpath);
                nh.getParam("/cnn/cfgpath", cfgpath); 
                nh.getParam("/cnn/classnamepath", classnamepath);
                nh.getParam("/cnn/set_confidence", set_confidence);
                nh.getParam("/cnn/input_type", input_type);
                nh.getParam("/cnn/CNN_size", CNN_size);
                nh.getParam("/cnn/w_GPU", w_GPU);


                CnnNodeletInitiate(cfgpath, weightpath, classnamepath);

                std::cout<<weightpath<<std::endl;
                std::cout<<cfgpath<<std::endl;
                std::cout<<classnamepath<<std::endl;
                std::cout<<set_confidence<<std::endl;

                //subscribe
                switch (input_type)
                {
                case 0: 
                // only rgb_raw
                    rgb_sub = nh.subscribe<sensor_msgs::Image>
                    (configs.getTopicName(COLOR_RAW_SUB_TOPIC), 1, &CnnNodelet::color_image_raw_callback, this);
                    break;
                case 1:
                // only rgb_raw_compressed
                    std::cout<<configs.getTopicName(COLOR_COMP_SUB_TOPIC)<<std::endl;
                    rgb_sub = nh.subscribe<sensor_msgs::CompressedImage>
                    (configs.getTopicName(COLOR_COMP_SUB_TOPIC), 1, &CnnNodelet::color_image_compressed_callback, this);
                    break;
                case 2:
                // rgb_raw + depth_raw
                    subimage_raw.subscribe(nh, configs.getTopicName(COLOR_RAW_SUB_TOPIC), 1);
                    subdepth_raw.subscribe(nh, configs.getTopicName(DEPTH_RAW_SUB_TOPIC), 1);                
                    sync_raw_.reset(new sync_raw( MySyncPolicy_RAW(10), subimage_raw, subdepth_raw));            
                    sync_raw_->registerCallback(boost::bind(&CnnNodelet::color_depth_image_raw_callback, this, _1, _2));
                
                    break;
                case 3:
                // rgb_compressed + depth_compressed
                    subimage_comp.subscribe(nh, configs.getTopicName(COLOR_COMP_SUB_TOPIC), 1);
                    subdepth_comp.subscribe(nh, configs.getTopicName(DEPTH_RAW_SUB_TOPIC), 1);                
                    sync_comp_.reset(new sync_comp( MySyncPolicy_COMP(10), subimage_comp, subdepth_comp));            
                    sync_comp_->registerCallback(boost::bind(&CnnNodelet::color_depth_image_compressed_callback, this, _1, _2));
                
                    break;
                default:
                    break;
                }
                

                image_transport::ImageTransport image_transport_(nh);

                pubimage = image_transport_.advertise(configs.getTopicName(IMAGE_PUB_TOPIC),1);


                ROS_INFO("CNN Nodelet Initiated...");
            }

    };

   PLUGINLIB_EXPORT_CLASS(mission_control_center::CnnNodelet, nodelet::Nodelet)

}

#endif