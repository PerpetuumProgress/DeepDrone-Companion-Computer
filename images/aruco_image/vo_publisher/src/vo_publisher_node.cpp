/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: pcd_to_pointcloud.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

/**

\author Ich bis Radu Bogdan Rusu

@b pcd_to_pointcloud is a simple node that loads PCD (Point Cloud Data) files from disk and publishes them as ROS messages on the network.

 **/

// ROS core
#include <ros/ros.h>
#include <Eigen/Geometry>   
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fiducial_msgs/FiducialTransform.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>


#include <string>
#include <sstream>
#include <vector>

// helper function to return parsed parameter or default value
template <typename T>
T get_param(std::string const& name, T default_value) {
    T value;
    ros::param::param<T>(name, value, default_value);
    return value;
}

static std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > sourceClouds;
static std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > transformedClouds;
static std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > emptyClouds;

class pcd_to_pointcloud {
    ros::NodeHandle nh;
    // the topiry_load_pointcloud()c to publish at, will be overwritten to give the remapped name
    std::string cloud_topic;

    // republish interval in seconds
    double interval;
    // tf2 frame_id
    std::string frame_id;
    // latched topic enabled/disabled
    bool latch;
    // pointcloud message
    sensor_msgs::PointCloud2 cloud;

    //subcriber and publisher
    ros::Publisher pub;
    ros::Subscriber sub;
    // timer to handle republishing
    ros::Timer timer;

    //tf2_ros::Buffer tfBuffer;
    //tf2_ros::TransformListener tfListener;

    void clearTransformedPointcloud(){
        for (int i=0; i<transformedClouds.size();i++){
            pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
            transformedClouds[i]=tempCloud;
        }
    }



	void publish() {
        
   	    pcl::PointCloud<pcl::PointXYZ>::Ptr mergerdPointcloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        for (int i=0; i<transformedClouds.size();i++){
            *mergerdPointcloud_ptr+=*transformedClouds[i];
            ROS_INFO("Added PointCloud");
        }

        pcl::toROSMsg (*mergerdPointcloud_ptr, cloud);
        cloud.header.frame_id = frame_id;

        ROS_DEBUG_STREAM_ONCE("Publishing pointcloud");
        ROS_DEBUG_STREAM_ONCE(" * number of points: " << cloud.width * cloud.height);
        ROS_DEBUG_STREAM_ONCE(" * frame_id: " << cloud.header.frame_id);
        ROS_DEBUG_STREAM_ONCE(" * topic_name: " << cloud_topic);
        int num_subscribers = pub.getNumSubscribers();
        if (num_subscribers > 0) {
            ROS_DEBUG("Publishing data to %d subscribers.", num_subscribers);
        }
        // update timestamp and publish
        cloud.header.stamp = ros::Time::now();
        pub.publish(cloud);
        clearTransformedPointcloud();
    }

    void timer_callback(ros::TimerEvent const&) {
        // just re-publish
        publish();
    }
	
    static void fiducialCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
        //Transform
        //
        //judge the lenght of the array
     
        
        
        for (int i=0; i < msg->transforms.size();i++){

            Eigen::Matrix4f aruco_to_camera_transform = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f yrotation = Eigen::Matrix4f::Identity();


            int fiducial_id = msg->transforms[i].fiducial_id;
            //Rotation Quarterion
            double q0 = msg->transforms[i].transform.rotation.w;
            double q1 = msg->transforms[i].transform.rotation.x;
            double q2 = msg->transforms[i].transform.rotation.y;
            double q3 = msg->transforms[i].transform.rotation.z;
            
            //Translation Vector
            double x = msg->transforms[i].transform.translation.x;
            double y = msg->transforms[i].transform.translation.y;
            double z = msg->transforms[i].transform.translation.z;
            



            
            // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
            aruco_to_camera_transform <<   2*(q0*q0+q1*q1)-1    ,2*(q1*q2-q0*q3)   ,2*(q1*q3+q0*q2)   , x ,   
       		                           2*(q1*q2+q0*q3)      ,2*(q0*q0+q2*q2)-1 ,2*(q2*q3-q0*q1)   , y ,    
                                           2*(q1*q3-q0*q2)      ,2*(q2*q3+q0*q1)   ,2*(q0*q0+q3*q3)-1 , z ,   
                                           0                    ,0                 ,0                 , 1 ;


            yrotation <<  0,0,1,0,
                          0,1,0,0,
                          -1,0,0,0,
                          0,0,0,1;

	    if(sourceClouds.size() >= msg->transforms[i].fiducial_id){
                pcl::transformPointCloud (*sourceClouds[msg->transforms[i].fiducial_id],*transformedClouds[msg->transforms[i].fiducial_id], aruco_to_camera_transform );
                pcl::transformPointCloud (*transformedClouds[msg->transforms[i].fiducial_id],*transformedClouds[msg->transforms[i].fiducial_id], yrotation );
            }

            //TRANSFORM TO LOCAL_ORIGIN FRAME HERE

            // Get the TF transform
           /*
            try {
              geometry_msgs::msg::TransformStamped tf_transform;
              tf_transform = tfBuffer.lookupTransform("local_origin", "camera_link", ros::Time(0));

	          Eigen::Matrix4f camera_to_localorigin_transform;

              transformAsMatrix(tf_transform, camera_to_localorigin_transform);
              Eigen::Matrix4f total_transform = aruco_to_camera_transform * camera_to_localorigin_transform;
              pcl::transformPointCloud (*transformedClouds[msg->transforms[i].fiducial_id],*transformedClouds[msg->transforms[i].fiducial_id], total_transform);

            } catch (tf2::TransformException &ex) {
              ROS_WARN("%s",ex.what());
              ros::Duration(1.0).sleep();
              continue;
            }
            */
 
            ROS_INFO("%.i ID: x=%.2f",msg->transforms[i].fiducial_id ,msg->transforms[i].transform.translation.x);
            ROS_INFO("%.i ID: z=%.2f",msg->transforms[i].fiducial_id ,msg->transforms[i].transform.translation.z);
            ROS_INFO("%.i ID: y=%.2f",msg->transforms[i].fiducial_id ,msg->transforms[i].transform.translation.y);
        }
    }
/*
    void transformAsMatrix(const tf2::Transform & bt, Eigen::Matrix4f & out_mat) {
        double mv[12];
        bt.getBasis().getOpenGLSubMatrix(mv);

        tf2::Vector3 origin = bt.getOrigin();

        out_mat(0, 0) = mv[0]; out_mat(0, 1) = mv[4]; out_mat(0, 2) = mv[8];
        out_mat(1, 0) = mv[1]; out_mat(1, 1) = mv[5]; out_mat(1, 2) = mv[9];
        out_mat(2, 0) = mv[2]; out_mat(2, 1) = mv[6]; out_mat(2, 2) = mv[10];

        out_mat(3, 0) = out_mat(3, 1) = out_mat(3, 2) = 0; out_mat(3, 3) = 1;
        out_mat(0, 3) = origin.x();
        out_mat(1, 3) = origin.y();
        out_mat(2, 3) = origin.z();
    }

    void transformAsMatrix(const geometry_msgs::msg::TransformStamped & bt, Eigen::Matrix4f & out_mat) {
        tf2::Transform transform;
        tf2::convert(bt.transform, transform);
        transformAsMatrix(transform, out_mat);
    }
*/


public:
    pcd_to_pointcloud()
    : cloud_topic("cloud_pcd"), interval(1), frame_id("camera_link"), latch(false)
    {
        // update potentially remapped topic name for later logging
        cloud_topic = nh.resolveName(cloud_topic);
        //tfListener = new TransformListener(tfBuffer);
    }



    void parse_ros_params() {
	    interval = get_param("~interval", interval);
        frame_id = get_param("~frame_id", frame_id);
        latch = get_param("~latch", latch);
    }

    void parse_cmdline_args(int argc, char** argv) {
        if (argc > 1) {
            std::stringstream str(argv[1]);
            double x;
            if (str >> x)
                interval = x;
        }
    }

    bool try_load_pointcloud() {
        std::string file_name;
        //save PointClouds to array
        for (int i = 0; i < 120; i++)
        {
            file_name="/root/catkin_ws/src/vo_publisher/resources/"+std::to_string(i)+".pcd";
            pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);

            if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *sourceCloud) != 0)
            {
                return -1;
            }
            ROS_INFO("Loaded file %.i",i);
            sourceClouds.push_back(sourceCloud);
            transformedClouds.push_back(transformedCloud);
            //cout << "Point Cloud " << i-1 << "has got " << sourceClouds[i-1]->size() << " Points" << endl;
        }
  	    return true;
    }

    void init_run() {
	    // init subscriber
	    sub = nh.subscribe("fiducial_transforms",100 , fiducialCallback);
        // init publisher                                                     
        pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, 1, latch);
        // treat publishing once as a special case to interval publishing
        
        bool oneshot = interval <= 0;
        timer = nh.createTimer(ros::Duration(interval),
                               &pcd_to_pointcloud::timer_callback,
                               this,
                               oneshot);
                               
    }

    void print_config_info() {
        ROS_INFO_STREAM("Recognized the following parameters");
        ROS_INFO_STREAM(" * interval: " << interval);
        ROS_INFO_STREAM(" * frame_id: " << frame_id);
        ROS_INFO_STREAM(" * topic_name: " << cloud_topic);
        ROS_INFO_STREAM(" * latch: " << std::boolalpha << latch);
    }

    void print_data_info() {
        ROS_INFO_STREAM("Loaded pointcloud with the following stats");
        ROS_INFO_STREAM(" * number of points: " << cloud.width * cloud.height);
        ROS_INFO_STREAM(" * total size [bytes]: " << cloud.data.size());
        ROS_INFO_STREAM(" * channel names: " << pcl::getFieldsList(cloud));
    }
};

int main (int argc, char** argv) {
    // init ROS
    ros::init(argc, argv, "pcd_to_pointcloud");
    ROS_INFO("Initialized");
    // set up node
    pcd_to_pointcloud node;
    // initializes from ROS parameters
    ROS_INFO("Node created");
    node.parse_ros_params();
    // also allow config to be provided via command line args
    // the latter take precedence
    ROS_INFO("rosparams parsed");
    node.parse_cmdline_args(argc, argv);
    // print info about effective configuration settings
    ROS_INFO("cmd arguments parsed");
    node.print_config_info();
    // try to load pointcloud from file
    if (!node.try_load_pointcloud()) {
        return -1;
    }
    ROS_INFO("loaded Pointcloud");
    // print info about pointcloud
   // node.print_data_info();
    // initialize run
    node.init_run();
    // blocking call to process callbacks etc
    ros::spin();
}
