#include <ros/ros.h>
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <types.h>
#include <m-detector/DynObjFilter.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <pcl/filters/random_sample.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

#include <deque>

// #include "preprocess.h"

using namespace std;

shared_ptr<DynObjFilter> DynObjFilt(new DynObjFilter());
M3D cur_rot = Eigen::Matrix3d::Identity();
V3D cur_pos = Eigen::Vector3d::Zero();

int     QUAD_LAYER_MAX  = 1;
int     occlude_windows = 3;
int     point_index = 0;
float   VER_RESOLUTION_MAX  = 0.01;
float   HOR_RESOLUTION_MAX  = 0.01;
float   angle_noise     = 0.001;
float   angle_occlude     = 0.02;
float   dyn_windows_dur = 0.5;
bool    dyn_filter_en = true, dyn_filter_dbg_en = true;
string  points_topic, odom_topic;
string  out_folder, out_folder_origin;
double  lidar_end_time = 0;
int     dataset = 0;
int     cur_frame = 0;

deque<M3D> buffer_rots;
deque<V3D> buffer_poss;
deque<double> buffer_times;
deque<boost::shared_ptr<PointCloudXYZI>> buffer_pcs;


ros::Publisher pub_pcl_dyn, pub_pcl_dyn_extend, pub_pcl_std; 

void OdomCallback(const nav_msgs::Odometry &cur_odom)
{
    // static int count = 0;
    // count++;
    // if (count % 10 != 0)
    // {
    //     return;
    // }
    std::cout << "OdomCallbackh " << std::endl;
    Eigen::Quaterniond cur_q;
    geometry_msgs::Quaternion tmp_q;
    tmp_q = cur_odom.pose.pose.orientation;
    tf::quaternionMsgToEigen(tmp_q, cur_q);
    cur_rot = cur_q.matrix();
    cur_pos << cur_odom.pose.pose.position.x, cur_odom.pose.pose.position.y, cur_odom.pose.pose.position.z;
    buffer_rots.push_back(cur_rot);
    buffer_poss.push_back(cur_pos);
    lidar_end_time = cur_odom.header.stamp.toSec();
    buffer_times.push_back(lidar_end_time);
}

void PoseCallback(const geometry_msgs::PoseStamped &cur_pose)
{
    Eigen::Quaterniond cur_q;
    geometry_msgs::Quaternion tmp_q;
    tmp_q = cur_pose.pose.orientation;
    tf::quaternionMsgToEigen(tmp_q, cur_q);
    cur_rot = cur_q.matrix();
    cur_pos << cur_pose.pose.position.x, cur_pose.pose.position.y, cur_pose.pose.position.z;
    buffer_rots.push_back(cur_rot);
    buffer_poss.push_back(cur_pos);
    lidar_end_time = cur_pose.header.stamp.toSec();
    buffer_times.push_back(lidar_end_time);
}

struct HesaiPointXYZIT
{
    PCL_ADD_POINT4D;
    uint8_t intensity;
    double timestamp = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(HesaiPointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(double, timestamp, timestamp))


void PointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in)
{
    pcl::PointCloud<HesaiPointXYZIT>::Ptr raw_pts(new pcl::PointCloud<HesaiPointXYZIT>());
    pcl::fromROSMsg(*msg_in, *raw_pts);
    boost::shared_ptr<PointCloudXYZI> feats_undistort(new PointCloudXYZI());
    pcl::fromROSMsg(*msg_in, *feats_undistort);
    buffer_pcs.push_back(feats_undistort);
    return;

    /*
    pcl::PointCloud<HesaiPointXYZIT>::Ptr raw_pts(new pcl::PointCloud<HesaiPointXYZIT>());
    pcl::fromROSMsg(*msg_in, *raw_pts);
    for (size_t i = 0; i < raw_pts->size(); i++)
    {
        if( std::fabs(raw_pts->points[i].x) < 3.0 && std::fabs(raw_pts->points[i].y) < 3.0  )
        {
             continue;
        }
        PointType po;
        po.x = raw_pts->points[i].x;
        po.y = raw_pts->points[i].y;
        po.z = raw_pts->points[i].z;
        po.intensity = raw_pts->points[i].intensity;
        feats_undistort->push_back(po);
    }
    feats_undistort->header.stamp = msg_in->header.stamp.toNSec() ; //   / 1000ull;
    feats_undistort->header.frame_id = msg_in->header.frame_id;
    feats_undistort->is_dense = msg_in->is_dense;
    feats_undistort->width = feats_undistort->points.size();
    feats_undistort->height = 1;
    buffer_pcs.push_back(feats_undistort);
    */

}


void TimerCallback(const ros::TimerEvent& e)
{
    if(buffer_pcs.size() > 0 && buffer_poss.size() > 0 && buffer_rots.size() > 0 && buffer_times.size() > 0)
    {
        boost::shared_ptr<PointCloudXYZI> cur_pc_boost = buffer_pcs.at(0);
        buffer_pcs.pop_front();
        // Convert boost::shared_ptr to std::shared_ptr
        PointCloudXYZI::Ptr cur_pc = std::make_shared<PointCloudXYZI>(*cur_pc_boost);
        auto cur_rot = buffer_rots.at(0);
        buffer_rots.pop_front();
        auto cur_pos = buffer_poss.at(0);
        buffer_poss.pop_front();
        auto cur_time = buffer_times.at(0);
        buffer_times.pop_front();
        string file_name = out_folder;
        stringstream ss;
        ss << setw(6) << setfill('0') << cur_frame ;
        file_name += ss.str(); 
        file_name.append(".label");
        string file_name_origin = out_folder_origin;
        stringstream sss;
        sss << setw(6) << setfill('0') << cur_frame ;
        file_name_origin += sss.str(); 
        file_name_origin.append(".label");

        if(file_name.length() > 15 || file_name_origin.length() > 15)
            DynObjFilt->set_path(file_name, file_name_origin);

        DynObjFilt->filter(cur_pc, cur_rot, cur_pos, cur_time);
        DynObjFilt->publish_dyn(pub_pcl_dyn, pub_pcl_dyn_extend, pub_pcl_std, cur_time);
        cur_frame ++;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynfilter_odom");
    ros::NodeHandle nh;
    nh.param<string>("dyn_obj/points_topic", points_topic, "");
    nh.param<string>("dyn_obj/odom_topic", odom_topic, "");
    nh.param<string>("dyn_obj/out_file", out_folder,"");
    nh.param<string>("dyn_obj/out_file_origin", out_folder_origin, "");

    DynObjFilt->init(nh);    
    /*** ROS subscribe and publisher initialization ***/
    pub_pcl_dyn_extend = nh.advertise<sensor_msgs::PointCloud2>("/m_detector/dynamic_frame_out", 10000);  
    pub_pcl_dyn = nh.advertise<sensor_msgs::PointCloud2> ("/m_detector/point_out", 100000);
    pub_pcl_std  = nh.advertise<sensor_msgs::PointCloud2> ("/m_detector/std_points", 100000);   
    ros::Subscriber sub_pcl = nh.subscribe(points_topic, 200000, PointsCallback);
    ros::Subscriber sub_odom = nh.subscribe(odom_topic, 200000, OdomCallback);
    ros::Subscriber sub_pose = nh.subscribe("pose_offline", 200000, PoseCallback);
    ros::Subscriber sub_pcl_offline = nh.subscribe("points_offline", 200000, PointsCallback);
    
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), TimerCallback);

    ros::spin();
    return 0;
}
