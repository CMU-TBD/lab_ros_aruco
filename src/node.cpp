

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <map>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"

#include "aruco.h"
#include "lab_ros_aruco/DetectedMarkers.h"
#include "lab_ros_aruco/Marker.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

//using namespace std;

class AruCoProcessing
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::CameraSubscriber cam_sub_;
    ros::Publisher markers_pub_;
    aruco::MarkerDetector detector_;
    float marker_size_meters_;
    std::string dictionary_name_;
    std::map<int, aruco::MarkerPoseTracker> tracker_map_;
    
public:
    AruCoProcessing()
        : it_(nh_){
        //load parameters
        if(!nh_.getParam("/aruco_node/marker_dictonary",dictionary_name_))
            dictionary_name_ = "ARUCO_MIP_36h12";
        if(!nh_.getParam("/aruco_node/marker_size",marker_size_meters_))
            marker_size_meters_ = 0.053;

        //set local parameters
        detector_.setDictionary(dictionary_name_);
        //subscribe to local channels 
        cam_sub_ = it_.subscribeCamera("/image",1,
        &AruCoProcessing::imageCb, this);
        markers_pub_ = nh_.advertise<lab_ros_aruco::DetectedMarkers>("markers",1);
        ROS_INFO("Starting Aruco Detection library with marker type: %s", dictionary_name_.c_str());
    }

    aruco::CameraParameters _convertCameraInfo(const sensor_msgs::CameraInfoConstPtr& cam_info){
        
        float cam_data[9];
        for(int i = 0; i < 9;i++){
            cam_data[i] = cam_info->K[i];
        }
        cv::Mat cam_mat(3,3,5,cam_data);
        
        float distortion_data[] = {0,0,0,0,0};
        for(int i = 0; i < 5;i++){
            distortion_data[i] = cam_info->D[i];
        }
        cv::Mat distortian_mat(1,5,5,distortion_data);

        cv::Size cam_size(cam_info->width, cam_info->height);
        
        return aruco::CameraParameters(cam_mat,distortian_mat,cam_size);
    }

    geometry_msgs::PoseStamped convertToStampedPose(std_msgs::Header header, cv::Mat rt_mat){
        geometry_msgs::PoseStamped pose;
        pose.header = header;

        tf::Matrix3x3 rotm(
            rt_mat.at<float>(0,0),rt_mat.at<float>(0,1),rt_mat.at<float>(0,2),
            rt_mat.at<float>(1,0),rt_mat.at<float>(1,1),rt_mat.at<float>(1,2),
            rt_mat.at<float>(2,0),rt_mat.at<float>(2,1),rt_mat.at<float>(2,2)
        );

        tf::Vector3 tv(
            rt_mat.at<float>(0,3),
            rt_mat.at<float>(1,3),
            rt_mat.at<float>(2,3)
        );
        tf::poseTFToMsg(tf::Transform(rotm, tv),pose.pose);
        return pose;
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info){
        //std::cout << "image callback";
        ROS_DEBUG("Hello1");
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
        ROS_DEBUG("Hello2");        
        auto image_raw = cv_ptr->image;
        auto cam_parameter = _convertCameraInfo(cam_info);
        auto markers = detector_.detect(image_raw);
        ROS_DEBUG("Found %d tags",markers.size());
        if(markers.size() > 0){
            lab_ros_aruco::DetectedMarkers detected_markers_msgs;            
            for (auto &marker : markers){
                //track pose
                auto success = tracker_map_[marker.id].estimatePose(marker, cam_parameter, marker_size_meters_, 1.0f);
                if(success){
                    auto rt_mat = tracker_map_[marker.id].getRTMatrix();
                    //std::cout << tracker_map_[marker.id].getTvec() << std::endl;
                    auto pose = convertToStampedPose(msg->header, rt_mat);
                    lab_ros_aruco::Marker marker_msg;
                    marker_msg.id = marker.id;
                    marker_msg.pose = pose;
                    marker_msg.tag_dict_name = dictionary_name_;
                    detected_markers_msgs.markers.push_back(marker_msg);
                }
                //bool success = marker.calculateExtrinsics(marker_size_meters_,cam_parameter,1.0f);
            }
            markers_pub_.publish(detected_markers_msgs);
        }
    }
};



// main() is where program execution begins.
int main(int argc, char** argv ) {

    ros::init(argc, argv,"aruco_process_node");
    try{
        AruCoProcessing pc;
        ROS_DEBUG("Spinning");
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("exception: %s", e.what());
        return 0;
    }
   
    //cout << "Hello World"; // prints Hello World
   //return 0;
}
