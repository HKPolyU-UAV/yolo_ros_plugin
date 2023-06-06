/*
    This file is part of ALan - the non-robocentric dynamic landing system for quadrotor

    ALan is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ALan is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ALan.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file yolo.cpp
 * \date 01/07/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief //legacy of AUTO (https://github.com/HKPolyU-UAV/AUTO) & previous 1st gen ALan (https://www.mdpi.com/1424-8220/22/1/404)
 */

#include "include/yolo.h"

void mission_control_center::CnnNodelet::camera_callback(const sensor_msgs::ImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth)
{
    if(intiated)
    {
        std::cout<<1<<std::endl;
        cv_bridge::CvImageConstPtr depth_ptr, rgb_ptr;
        try
        {
            depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat image_dep = depth_ptr->image;

        this->getdepthdata(image_dep);

        std::cout<<2<<std::endl;

        try
        {
            // this->frame = cv::imdecode(cv::Mat(rgbimage->data),1);
            rgb_ptr  = cv_bridge::toCvCopy(rgbimage, rgbimage->encoding);
            // res   = cv::imdecode(cv::Mat(rgbimage->data),1);
            // gt    = cv::imdecode(cv::Mat(rgbimage->data),1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        frame = rgb_ptr->image;

        std::cout<<3<<std::endl;
        
        // std::cout<<frame.size()<<std::endl;

        char hz[40];
        char fps[5] = " fps";
        sprintf(hz, "%.2f", this->appro_fps);
        strcat(hz, fps);
        cv::putText(frame, hz, cv::Point(20,40), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));
        std::cout<<4<<std::endl;
        std::cout<<"hi here is the hz: "<<hz<<std::endl;
        
        rundarknet(this->frame);
        ROS_INFO("YOLO");

        cv::Rect ROI;

        for(auto what : obj_vector)
        {
            ROI = what.boundingbox;
        }
        
        // cv::Mat test(frame.rows, frame.cols, CV_8UC3, CV_RGB(255, 255, 255));
        
        // cv::rectangle(test, ROI, CV_RGB(0, 0, 0), -1);

        // // std::cout<<frame.type()<<std::endl;
        // // std::cout<<test.type()<<std::endl;
        // // std::cout<<frame.size()<<std::endl;
        // // std::cout<<test.size()<<std::endl;

        // cv::subtract(frame, test, frame);

        // cv::Mat imageoutput = frame.clone();
        // cv_bridge::CvImage for_visual;
        // for_visual.header = rgbimage->header;
        // for_visual.encoding = sensor_msgs::image_encodings::BGR8;
        // for_visual.image = imageoutput;
        // this->pubimage.publish(for_visual.toImageMsg());
        // ywy
        if(obj_vector.size() > 0){

            if (obj_vector[0].confidence < 30) {
                std::cout << "the crack confidence is lower than 30%, discarded" << std::endl;
                return;
            }
            
            cv::Mat imageoutput = frame.clone();
            cv_bridge::CvImage for_visual;
            for_visual.header = rgbimage->header;
            for_visual.encoding = sensor_msgs::image_encodings::BGR8;
            for_visual.image = imageoutput;

            double about_to_pub_time = ros::Time::now().toSec();

            double time_gap_now = about_to_pub_time - last_pub_time_;
            if (time_gap_now > time_gap_) {
                this->pubimage.publish(for_visual.toImageMsg());  
            } else {
                std::cout << "the time gap between 2 frames are too small" << std::endl;
                std::cout << "we are NOT going to pub this frame" << std::endl;
            }
            // /red/camera/depth/image_raw
            // we need to subscribe this 

            // this->pubimage.publish(for_visual.toImageMsg());  
        } else {
            std::cout << "there is no crack detected" << std::endl;
        }
        // ywy
        
        
    }
    // display(this->frame);
    // cv::waitKey(10);

    // ros::Duration(5.0).sleep();
    // std::cout<<frame.size<<std::endl;
}

inline void mission_control_center::CnnNodelet::CnnNodeletInitiate(const cv::String cfgfile, const cv::String weightfile, const cv::String classnamepath)
{
    std::cout<<"start initiation..."<<std::endl;
    this->mydnn = cv::dnn::readNetFromDarknet(cfgfile, weightfile);

    //opt CUDA or notcc
    // this->mydnn.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
    // this->mydnn.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    
    // ywy
    last_pub_time_ = ros::Time::now().toSec();

    intiated = true;

    std::cout<<"end   initiation..."<<std::endl;
    
    this->mydnn.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    this->mydnn.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

}

void mission_control_center::CnnNodelet::rundarknet(cv::Mat &frame)
{
    this->obj_vector.clear();
    this->total_start = std::chrono::steady_clock::now();
    findboundingboxes(frame);
    this->total_end = std::chrono::steady_clock::now();
    total_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();
    this->appro_fps = total_fps;
}

void mission_control_center::CnnNodelet::display(cv::Mat frame)
{
    // cv::imshow("Yolo-ed", frame);
    // cv::waitKey(20);
}

void mission_control_center::CnnNodelet::getdepthdata(cv::Mat depthdata)
{
    this->depthdata = depthdata;
}

void mission_control_center::CnnNodelet::findboundingboxes(cv::Mat &frame)
{
    cv::Mat blob;
    using namespace std;

    cout<<frame.size<<endl;

    cout<<1<<endl;
    blob = cv::dnn::blobFromImage(frame, 0.00392, cv::Size(608, 608), cv::Scalar(), true, false);
    //    as the dnn::net function does not accept image from image, it only receive blob hence the above function, refer to teams/article/blob

    cout<<2<<endl;
    mydnn.setInput(blob);
    //feed 4-D blob to darknet dnn

    std::vector<cv::String> net_outputNames;//names of output layer of yolo, should be 3
    net_outputNames = mydnn.getUnconnectedOutLayersNames();

    std::vector<cv::Mat> netOutput;
    //coppy the result to this object


    double starttime = ros::Time::now().toSec();

    mydnn.forward(netOutput, net_outputNames);

    //    std::cout<<netOut[0].size()<<std::endl;
    //    std::cout<<netOut[1].size()<<std::endl;
    //    std::cout<<netOut[2].size()<<std::endl<<std::endl;
    double endtime = ros::Time::now().toSec();
    double deltatime = endtime - starttime;
    // std::cout<<"time:"<<deltatime<<std::endl;
    // std::cout<<"fps: "<<1/deltatime<<std::endl;
    findwhichboundingboxrocks(netOutput, frame);
}

void mission_control_center::CnnNodelet::findwhichboundingboxrocks(std::vector<cv::Mat> &netOutput, cv::Mat &frame)
{
    std::vector<float> confidenceperbbox;
    std::vector<int> indices;
    std::vector<cv::Rect> bboxes;
    std::vector<std::string> classnames;
    std::vector<int> classids;

    getclassname(classnames);

    int indicator =0;
    for(auto &output: netOutput)//read every layer's output, the auto variable of "output" indicates 3 different layer, as per the architecture of yolo
    {
        for(int i=0;i<output.rows;i++)//now, for every layer's output, there will be 17328*(5+class number), 4332*(5+class number), 1083*(5+class number) numbers, it holds the very info of every predicted bounding boxes
        {
            auto isthereanobjectconfidence = output.at<float> (i,4);//save the confidence of every bounding box
            if(isthereanobjectconfidence>set_confidence)//this does: assess whether there is an object in this bounding box
            //if there is, further extract the data
            {
                auto x =output.at<float>(i,0) * frame.cols;
                auto y =output.at<float>(i,1) * frame.rows;
                auto w =output.at<float>(i,2) * frame.cols;
                auto h =output.at<float>(i,3) * frame.rows;
                //                auto c_max =output.at<float>(i,4);
                //                auto c_no1 =output.at<float>(i,5);
                //                auto c_no2 =output.at<float>(i,6);
                //                auto c_no3 =output.at<float>(i,7);
                //                auto c_no4 =output.at<float>(i,8);
                //                auto c_no5 =output.at<float>(i,9);
                auto x_ = int(x - w/2);
                auto y_ = int(y - h/2);
                auto w_ = int(w);
                auto h_ = int(h);
                cv::Rect Rect_temp(x_,y_,w_,h_);

                //                std::cout<<"here: "<<c_max<<" "<<c_no1<<" "<<c_no2<<" "<<c_no3<<" "<<c_no4<<" "<<c_no5<<" "<<std::endl<<std::endl;
                for(int class_i=0;class_i<classnames.size();class_i++)//as for this step, this for loop take the probabilities of every class
                {
                    auto confidence_each_class = output.at<float>(i, 5+class_i); //6th element will be the 1st class confidence, class id=0
                        //7th element will be the 2nd class confidence, class id=1, etc
                    if(confidence_each_class>set_confidence)
                    {
                        bboxes.push_back(Rect_temp);
                        confidenceperbbox.push_back(confidence_each_class);
                        classids.push_back(class_i);
                    }
                }
            }
        }
    }

    cv::dnn::NMSBoxes(bboxes,confidenceperbbox,0.1,0.1,indices);
    //Basically, the indicies return the index of the bboxes, i.e, show which bounding box is the most suitable one
    for(int i =0 ; i < indices.size();i++)
    {
        int index = indices[i];

        int final_x, final_y, final_w, final_h;
        final_x = bboxes[index].x;
        final_y = bboxes[index].y;
        final_w = bboxes[index].width;
        final_h = bboxes[index].height;
        cv::Scalar color;

        cv::Point center = cv::Point(final_x+final_w/2, final_y+final_h/2);
        int depthbox_w = final_w*0.25;
        int depthbox_h = final_h*0.25;

        cv::Point depthbox_vertice1 = cv::Point(center.x - depthbox_w/2, center.y - depthbox_h/2);
        cv::Point depthbox_vertice2 = cv::Point(center.x + depthbox_w/2, center.y + depthbox_h/2);
        cv::Rect letsgetdepth(depthbox_vertice1, depthbox_vertice2);

        cv::Mat ROI(depthdata, letsgetdepth);
        cv::Mat ROIframe;
        ROI.copyTo(ROIframe);
        std::vector<cv::Point> nonzeros;

        cv::findNonZero(ROIframe, nonzeros);
        std::vector<double> nonzerosvalue;
        for(auto temp : nonzeros)
        {
            double depth = ROIframe.at<ushort>(temp);
            nonzerosvalue.push_back(depth);
        }

        double depth_average;
        if(nonzerosvalue.size()!=0)
            depth_average = accumulate(nonzerosvalue.begin(), nonzerosvalue.end(),0.0)/nonzerosvalue.size();
            // ywy
            std::cout << "THE DEPTH IS " << depth_average << "!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;            


        cv::Point getdepth(final_x+final_w/2, final_y+final_h/2);
        double depthofboundingbox = 0.001 * depth_average;
        int temp_iy = 0;


        std::string detectedclass = classnames[classids[index]];
        float detectedconfidence = confidenceperbbox[index]*100;

        char temp_depth[40];
        sprintf(temp_depth, "%.2f", depthofboundingbox);
        char temp_confidence[40];
        sprintf(temp_confidence, "%.2f", detectedconfidence);     


        std::string textoutputonframe = detectedclass + ": " + temp_confidence + "%, "+ temp_depth + "m";


        cv::Scalar colorforbox(rand()&255, rand()&255, rand()&255);

        cv::rectangle(frame, cv::Point(final_x, final_y), cv::Point(final_x+final_w, final_y+final_h), colorforbox,2);
        cv::putText(frame, textoutputonframe, cv::Point(final_x,final_y-10),cv::FONT_HERSHEY_COMPLEX_SMALL,1,CV_RGB(255,255,0));
        obj.confidence = detectedconfidence;
        obj.classnameofdetection = detectedclass;
        obj.boundingbox = cv::Rect(cv::Point(final_x, final_y), cv::Point(final_x+final_w, final_y+final_h));
        obj.depth = depthofboundingbox;
        obj.frame = frame;
        obj_vector.push_back(obj);
    }

}

void mission_control_center::CnnNodelet::getclassname(std::vector<std::string> &classnames)
{
    std::ifstream class_file(classnamepath);
    if (!class_file)
    {
        std::cerr << "failed to open classes.txt\n";
    }

    std::string line;
    while (getline(class_file, line))
    {
        classnames.push_back(line);
    }
}