

#include <ros/ros.h>
#include <ros/console.h>
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
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <mutex>
#include <std_msgs/Header.h>

//using namespace std;

class AruCoProcessing
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::CameraSubscriber cam_sub_;
    int process_rate_;
    ros::Publisher markers_pub_, poses_pub_;
    aruco::MarkerDetector detector_;
    float marker_size_meters_;
    std::string dictionary_name_;
    std::map<int, aruco::MarkerPoseTracker> tracker_map_;
    std::mutex image_mutex_;
    bool valid = false;
    cv::Mat image_;
    aruco::CameraParameters camera_info_;
    std_msgs::Header latest_header_;

  public:
    AruCoProcessing()
        : it_(nh_), nh_("AruCoMakerNode")
    {
        //load parameters
        if (!nh_.getParam("/aruco_node/marker_dictonary", dictionary_name_))
            dictionary_name_ = "ARUCO_MIP_36h12";
        if (!nh_.getParam("/aruco_node/marker_size", marker_size_meters_))
            marker_size_meters_ = 0.053;
        if (!nh_.getParam("/aruco_node/rate", marker_size_meters_))
            process_rate_ = 20;

        //set local parameters
        detector_.setDictionary(dictionary_name_);
        //subscribe to local channels
        cam_sub_ = it_.subscribeCamera("/image", 1,
                                       &AruCoProcessing::imageCb, this);
        markers_pub_ = nh_.advertise<lab_ros_aruco::DetectedMarkers>("markers", 1);
        poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("marker_poses", 1);
        ROS_INFO("ROS ARUCO: DICT: %s, size:%f", dictionary_name_.c_str(), marker_size_meters_);
    }

    aruco::CameraParameters _convertCameraInfo(const sensor_msgs::CameraInfoConstPtr &cam_info)
    {

        float cam_data[9];
        if (cam_info->K.size() >= 9)
        {
            for (int i = 0; i < 9; i++)
            {
                cam_data[i] = cam_info->K[i];
            }
        }
        else
        {
            ROS_ERROR("Not enough matrix entries (9 needed) in cam_info->K. Undefined behavior");
        }
        cv::Mat cam_mat(3, 3, 5, cam_data);

        float distortion_data[] = {0, 0, 0, 0, 0};
        if (cam_info->D.size() >= 5)
        {
            for (int i = 0; i < 5; i++)
            {
                distortion_data[i] = cam_info->D[i];
            }
        }
        else
        {
            ROS_ERROR("No distortion model params in the camera info message -- has this camera been calibrated?");
        }
        cv::Mat distortian_mat(1, 5, 5, distortion_data);

        cv::Size cam_size(cam_info->width, cam_info->height);

        return aruco::CameraParameters(cam_mat, distortian_mat, cam_size);
    }

    geometry_msgs::PoseStamped convertToStampedPose(std_msgs::Header header, cv::Mat rt_mat)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = header;

        //seperate out the rotation matrix
        tf::Matrix3x3 rotm(
            rt_mat.at<float>(0, 0), rt_mat.at<float>(0, 1), rt_mat.at<float>(0, 2),
            rt_mat.at<float>(1, 0), rt_mat.at<float>(1, 1), rt_mat.at<float>(1, 2),
            rt_mat.at<float>(2, 0), rt_mat.at<float>(2, 1), rt_mat.at<float>(2, 2));
        //the translation matrix
        tf::Vector3 tv(
            rt_mat.at<float>(0, 3),
            rt_mat.at<float>(1, 3),
            rt_mat.at<float>(2, 3));
        tf::poseTFToMsg(tf::Transform(rotm, tv), pose.pose);

        // tf::Quaternion axisChange, initialRotation;
        // tf::quaternionMsgToTF(pose.pose.orientation, initialRotation);
        // axisChange.setEuler(-1 * M_PI / 2.0, 0.0, 0.0); //rotate -90 degrees along the X axis
        // tf::quaternionTFToMsg(initialRotation * axisChange, pose.pose.orientation);

        return pose;
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &cam_info)
    {

        std::unique_lock<std::mutex> lock1(image_mutex_);

        image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        camera_info_ = _convertCameraInfo(cam_info);
        valid = true;
        latest_header_ = msg->header;
    }

    void processCB(const ros::TimerEvent &event)
    {
        //check to see if the process is running at the set speed (default 20Hz)
        if((1.0/process_rate_) < event.profile.last_duration.toSec()){
            ROS_WARN("Process is running behind set rate of %d real rate:%f",process_rate_, 1.0/(event.current_real - event.last_real).toSec());
        }

        cv::Mat image_raw;
        aruco::CameraParameters cam_parameter;
        std_msgs::Header header;
        //copy in the latest copy of the results
        {
            std::unique_lock<std::mutex> lock2(image_mutex_);
            if (!valid)
            {
                return;
            }
            image_raw = image_;
            cam_parameter = camera_info_;
            header = latest_header_;
            valid = false;
        }
        //process the latest image
        auto markers = detector_.detect(image_raw);
        ROS_INFO("Detected %lu markers", markers.size());

        if (markers.size() > 0)
        {
            lab_ros_aruco::DetectedMarkers detected_markers_msgs;
            geometry_msgs::PoseArray poseArray;
            poseArray.header = header;
            std::vector<geometry_msgs::Pose> poses;
            for (auto &marker : markers)
            {
                //track pose
                auto success = tracker_map_[marker.id].estimatePose(marker, cam_parameter, marker_size_meters_);
                if (success)
                {
                    auto rt_mat = tracker_map_[marker.id].getRTMatrix();
                    //std::cout << tracker_map_[marker.id].getTvec() << std::endl;
                    auto pose = convertToStampedPose(header, rt_mat);
                    lab_ros_aruco::Marker marker_msg;
                    marker_msg.id = marker.id;
                    marker_msg.pose = pose;
                    marker_msg.tag_dict_name = dictionary_name_;
                    detected_markers_msgs.markers.push_back(marker_msg);
                    poses.push_back(pose.pose);
                }
                else{
                    ROS_INFO("Failed to get Pose for tag:%d",marker.id);
                }
                //bool success = marker.calculateExtrinsics(marker_size_meters_,cam_parameter,1.0f);
            }
            markers_pub_.publish(detected_markers_msgs);
            poseArray.poses = poses;
            poses_pub_.publish(poseArray);
        }
    }
};

// main() is where program execution begins.
int main(int argc, char **argv)
{

    ros::init(argc, argv, "aruco_process_node");
    ros::NodeHandle nh;
    int rate = 20;
    if (ros::param::get("/aruco_node/rate", rate))
    {
        rate = 20;
    }

    try
    {
        AruCoProcessing pc;
        ROS_INFO("Spinning");
        ros::Timer timer1 = nh.createTimer(ros::Duration(1/rate), &AruCoProcessing::processCB, &pc);
        ROS_INFO("start spinning");
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("exception: %s", e.what());
        return 0;
    }
}
