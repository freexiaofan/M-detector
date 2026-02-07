#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <vector>
#include <string>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include "/home/xf/Desktop/catkin_ws/src/HBA/include/common.hpp"
#include "/home/xf/Desktop/catkin_ws/src/HBA/include/file_utils.hpp"
#include <algorithm>
#include <deque>
#include <types.h>

#include <m-detector/DynObjFilter.h>

using namespace std;

shared_ptr<DynObjFilter> DynObjFilt(new DynObjFilter());
M3D cur_rot = Eigen::Matrix3d::Identity();
V3D cur_pos = Eigen::Vector3d::Zero();

deque<M3D> buffer_rots;
deque<V3D> buffer_poss;
deque<double> buffer_times;
deque<boost::shared_ptr<PointCloudXYZI>> buffer_pcs;
deque<string> buffer_pcd_paths;

int cur_frame = 0;
string out_folder;
ros::Publisher pub_pcl_in;
ros::Publisher pub_pcl_out;


void OdomCallback(const nav_msgs::Odometry &cur_odom)
{
    auto tmp_ = cur_odom.pose.pose;
    Eigen::Quaterniond cur_q(tmp_.orientation.w, tmp_.orientation.x, tmp_.orientation.y, tmp_.orientation.z);
    Eigen::Matrix3d cur_rot = cur_q.matrix();
    Eigen::Vector3d cur_pos(tmp_.position.x, tmp_.position.y, tmp_.position.z);
    buffer_rots.push_back(cur_rot);
    buffer_poss.push_back(cur_pos);
    buffer_times.push_back(cur_odom.header.stamp.toSec());
}

void PointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in)
{
    boost::shared_ptr<PointCloudXYZI> feats_undistort(new PointCloudXYZI());
    pcl::fromROSMsg(*msg_in, *feats_undistort);
    buffer_pcs.push_back(feats_undistort);
}

static void saveStaticCloud(const PointCloudXYZI::Ptr &static_cloud, const string &file_path)
{
    if (!static_cloud)
        return;
    std::filesystem::path out_dir = std::filesystem::path(file_path).parent_path();
    if (!out_dir.empty())
        std::filesystem::create_directories(out_dir);
    pcl::io::savePCDFileBinary(file_path, *static_cloud);
}

static PointCloudXYZI::Ptr transformWorldToLidar(const PointCloudXYZI::Ptr &cloud_world,
                                                 const M3D &rot_end,
                                                 const V3D &pos_end)
{
    if (!cloud_world)
        return PointCloudXYZI::Ptr(new PointCloudXYZI());

    PointCloudXYZI::Ptr cloud_local(new PointCloudXYZI());
    cloud_local->reserve(cloud_world->size());
    const M3D rot_inv = rot_end.transpose();

    for (const auto &pt : cloud_world->points)
    {
        V3D pw(pt.x, pt.y, pt.z);
        V3D pl = rot_inv * (pw - pos_end);
        PointType out = pt;
        out.x = static_cast<float>(pl.x());
        out.y = static_cast<float>(pl.y());
        out.z = static_cast<float>(pl.z());
        cloud_local->points.push_back(out);
    }

    cloud_local->width = cloud_local->points.size();
    cloud_local->height = 1;
    cloud_local->is_dense = cloud_world->is_dense;
    return cloud_local;
}

static PointCloudXYZI::Ptr buildStaticCloudFromFilter()
{
    PointCloudXYZI::Ptr laserCloudSteadObj_pub(new PointCloudXYZI());
    if (DynObjFilt->cluster_coupled)
    {
        if (DynObjFilt->laserCloudSteadObj_clus)
        {
            *laserCloudSteadObj_pub = *DynObjFilt->laserCloudSteadObj_clus;
        }

        // PointCloudXYZI::Ptr laserCloudSteadObj_down(new PointCloudXYZI());
        // float leaf = DynObjFilt->voxel_filter_size;
        // ROS_INFO("static cloud voxel leaf size: %.3f", leaf);
        // if (leaf > 0.0f)
        // {
        //     pcl::VoxelGrid<PointType> downSizeFiltermap;
        //     downSizeFiltermap.setLeafSize(leaf, leaf, leaf);
        //     downSizeFiltermap.setInputCloud(laserCloudSteadObj_pub);
        //     downSizeFiltermap.filter(*laserCloudSteadObj_down);
        //     return laserCloudSteadObj_down;
        // }
        return laserCloudSteadObj_pub;
    }

    if (DynObjFilt->laserCloudSteadObj)
    {
        *laserCloudSteadObj_pub = *DynObjFilt->laserCloudSteadObj;
    }
    return laserCloudSteadObj_pub;
}

void TimerCallback()
{
    if (buffer_pcs.empty() || buffer_poss.empty() || buffer_rots.empty() || buffer_times.empty() || buffer_pcd_paths.empty())
        return;

    boost::shared_ptr<PointCloudXYZI> cur_pc_boost = buffer_pcs.front();
    buffer_pcs.pop_front();
    PointCloudXYZI::Ptr cur_pc = std::make_shared<PointCloudXYZI>(*cur_pc_boost);

    auto cur_rot = buffer_rots.front();
    buffer_rots.pop_front();
    auto cur_pos = buffer_poss.front();
    buffer_poss.pop_front();
    auto cur_time = buffer_times.front();
    buffer_times.pop_front();

    string pcd_path = buffer_pcd_paths.front();
    buffer_pcd_paths.pop_front();

    // publish input cloud (before filter)
    if (pub_pcl_in)
    {
        sensor_msgs::PointCloud2 msg_in;
        pcl::toROSMsg(*cur_pc, msg_in);
        msg_in.header.stamp = ros::Time().fromSec(cur_time);
        msg_in.header.frame_id = "lidar";
        pub_pcl_in.publish(msg_in);
    }

    DynObjFilt->filter(cur_pc, cur_rot, cur_pos, cur_time);

    PointCloudXYZI::Ptr static_cloud_world = buildStaticCloudFromFilter();
    PointCloudXYZI::Ptr static_cloud = transformWorldToLidar(static_cloud_world, cur_rot, cur_pos);
    std::filesystem::path p(pcd_path);
    string out_name = p.stem().string() + string(".pcd");
    string out_path = (std::filesystem::path(out_folder) / out_name).string();
    saveStaticCloud(static_cloud, out_path);

    // publish output cloud (after filter)
    if (pub_pcl_out && static_cloud)
    {
        sensor_msgs::PointCloud2 msg_out;
        pcl::toROSMsg(*static_cloud, msg_out);
        msg_out.header.stamp = ros::Time().fromSec(cur_time);
        msg_out.header.frame_id = "lidar";
        pub_pcl_out.publish(msg_out);
    }

    cur_frame++;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "remove_dynamic");
    ros::NodeHandle nh;

    bool strict_mode = true;
    nh.param("dyn_obj/strict_mode", strict_mode, true);
    if (strict_mode)
    {
        nh.setParam("dyn_obj/dyn_filter_en", true);
        nh.setParam("dyn_obj/cluster_coupled", true);
        nh.setParam("dyn_obj/cluster_future", true);
        nh.setParam("dyn_obj/cluster_min_pixel_number", 4);
        nh.setParam("dyn_obj/cluster_thrustable_thresold", 0.2);
        nh.setParam("dyn_obj/v_min_thr2", 0.5);
        nh.setParam("dyn_obj/acc_thr2", 3.0);
        nh.setParam("dyn_obj/v_min_thr3", 0.3);
        nh.setParam("dyn_obj/acc_thr3", 5.0);
        ROS_WARN("strict_mode enabled: dynamic filtering thresholds tightened");
    }

    std::string folder = "/media/xf/Elements/id4_1202/raw_data_3/"; // 可根据需要修改
    std::string points_folder = folder + "/pointclouds/"; 
    std::string odoms_folder = folder + "/odoms/"; 
    out_folder = folder + "/static_pointclouds/";

    std::vector<std::string> pcd_files = getFilesWithExtension(points_folder, ".pcd");
    ROS_INFO("Found %zu pcd files.", pcd_files.size());

    std::string tum_odom_file = folder + "/debug_file/opt_pose_utm.tum"; 
    std::vector<TumPose> tum_odoms = readTumPose(tum_odom_file);
    ROS_INFO("Found %zu tum_odoms files .", tum_odoms.size());

    DynObjFilt->init(nh);

    pub_pcl_in = nh.advertise<sensor_msgs::PointCloud2>("/remove_dynamic/input_cloud", 10);
    pub_pcl_out = nh.advertise<sensor_msgs::PointCloud2>("/remove_dynamic/output_cloud", 10);

    size_t total = std::min(pcd_files.size(), tum_odoms.size());
    // for (size_t i = 0; i < 100; i++)
    for (size_t i = 0; i < total; i++)
    {
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time().fromSec(tum_odoms[i].timestamp);
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "lidar";
        odom_msg.pose.pose.position.x = tum_odoms[i].t.x();
        odom_msg.pose.pose.position.y = tum_odoms[i].t.y();
        odom_msg.pose.pose.position.z = tum_odoms[i].t.z();
        odom_msg.pose.pose.orientation.x = tum_odoms[i].q.x();
        odom_msg.pose.pose.orientation.y = tum_odoms[i].q.y();
        odom_msg.pose.pose.orientation.z = tum_odoms[i].q.z();
        odom_msg.pose.pose.orientation.w = tum_odoms[i].q.w();
        OdomCallback(odom_msg);

        PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
        ROS_INFO("read file %s \n", pcd_files[i].c_str());
        if (pcl::io::loadPCDFile<PointType>(pcd_files[i], *cloud) == -1)
        {
            PCL_ERROR("Couldn't read file %s \n", pcd_files[i].c_str());
            continue;
        }
        // remove points within |x|<3 and |y|<3
        PointCloudXYZI::Ptr cloud_filtered(new PointCloudXYZI());
        cloud_filtered->reserve(cloud->size());
        for (const auto &pt : cloud->points)
        {
            if (std::fabs(pt.x) < 3.0f && std::fabs(pt.y) < 3.0f)
            {
                continue;
            }
            cloud_filtered->points.push_back(pt);
        }
        cloud_filtered->width = cloud_filtered->points.size();
        cloud_filtered->height = 1;
        cloud_filtered->is_dense = cloud->is_dense;
        cloud = cloud_filtered;
        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud, *cloud_msg);
        cloud_msg->header.stamp = ros::Time().fromSec(tum_odoms[i].timestamp);
        cloud_msg->header.frame_id = "lidar";
        PointsCallback(cloud_msg);

        buffer_pcd_paths.push_back(pcd_files[i]);
        TimerCallback();
    }

    return 0;
}

