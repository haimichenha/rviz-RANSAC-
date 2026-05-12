/**
 * @file pcd_clustering.cpp
 * @brief 实验三：实时障碍物聚类、边框显示及距离计算
 * 
 * 功能：
 * 1. 实时订阅激光雷达点云话题
 * 2. 使用欧几里得聚类算法对障碍物进行分类
 * 3. 边框跟随小车移动（使用雷达坐标系）
 * 4. 在 Rviz 中实时显示聚类边框和距离
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <vector>
#include <tuple>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// 全局发布器
ros::Publisher pub_clusters;
ros::Publisher pub_markers;

// 全局参数
std::string g_frame_id = "camera_init";  // 默认使用 camera_init

// 上一帧的 marker 数量
int last_marker_count = 0;

// 预定义鲜艳的颜色列表
std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> colors = {
    {255, 50, 50},    // 红色
    {50, 255, 50},    // 绿色
    {50, 100, 255},   // 蓝色
    {255, 255, 50},   // 黄色
    {255, 50, 255},   // 品红
    {50, 255, 255},   // 青色
    {255, 150, 50},   // 橙色
    {150, 50, 255},   // 紫色
    {50, 255, 150},   // 春绿
    {255, 50, 150},   // 玫红
    {150, 255, 50},   // 黄绿
    {100, 200, 255},  // 天蓝
};

/**
 * @brief 点云回调函数 - 实时处理每一帧激光雷达数据
 */
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg)
{
    // 转换 ROS 消息到 PCL 点云
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::fromROSMsg(*input_msg, *cloud);
    
    // 1. 坐标系和时间戳处理 (关键修复)
    // 强制使用点云消息自带的时间戳，确保与 bag 包同步
    ros::Time current_time = input_msg->header.stamp;
    if (current_time.isZero()) current_time = ros::Time::now();

    // 强制使用点云消息自带的 frame_id (通常是 rslidar 或 velodyne)
    // 这样聚类结果就在雷达局部坐标系下，自然会随车移动
    std::string frame_id = input_msg->header.frame_id;
    if (frame_id.empty()) frame_id = g_frame_id;

    ROS_INFO_THROTTLE(5.0, "[聚类] 收到点云: %lu 个点, 使用坐标系: %s, 时间戳: %.2f", 
                      cloud->size(), frame_id.c_str(), current_time.toSec());

    if (cloud->empty()) return;

    // 1. 体素降采样
    PointCloudT::Ptr cloud_downsampled(new PointCloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.2f, 0.2f, 0.2f);
    vg.filter(*cloud_downsampled);

    // 2. ROI 过滤 - 只处理小车周围一定范围内的点
    PointCloudT::Ptr cloud_roi(new PointCloudT);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_downsampled);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-20.0, 20.0);
    pass.filter(*cloud_roi);
    pass.setInputCloud(cloud_roi);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-20.0, 20.0);
    pass.filter(*cloud_roi);
    pass.setInputCloud(cloud_roi);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.0, 2.5);
    pass.filter(*cloud_roi);

    if (cloud_roi->empty()) return;

    // 3. 地面分割 (RANSAC)
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(50);
    seg.setDistanceThreshold(0.3);
    seg.setInputCloud(cloud_roi);
    seg.segment(*inliers, *coefficients);

    // 提取非地面点
    PointCloudT::Ptr cloud_obstacles(new PointCloudT);
    if (!inliers->indices.empty() && inliers->indices.size() > 100) {
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_roi);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_obstacles);
    } else {
        *cloud_obstacles = *cloud_roi;
    }

    // 4. 欧几里得聚类
    std::vector<pcl::PointIndices> cluster_indices;
    if (cloud_obstacles->size() > 30) {
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud_obstacles);
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.8);
        ec.setMinClusterSize(20);
        ec.setMaxClusterSize(5000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_obstacles);
        ec.extract(cluster_indices);
    }

    ROS_INFO_THROTTLE(3.0, "[聚类] 检测到 %lu 个障碍物", cluster_indices.size());

    // 5. 处理聚类并生成可视化
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    visualization_msgs::MarkerArray marker_array;
    
    // 清除上一帧的 marker
    visualization_msgs::Marker delete_all;
    delete_all.header.frame_id = frame_id;
    delete_all.header.stamp = current_time;
    delete_all.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);

    int cluster_id = 0;
    for (const auto& indices : cluster_indices) {
        PointCloudT::Ptr cluster(new PointCloudT);
        
        // 获取当前聚类的颜色
        auto& color = colors[cluster_id % colors.size()];
        uint8_t r = std::get<0>(color);
        uint8_t g = std::get<1>(color);
        uint8_t b = std::get<2>(color);
        
        // RGB 打包
        uint32_t rgb_packed = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        float rgb_float = *reinterpret_cast<float*>(&rgb_packed);

        float min_x = 1e9, min_y = 1e9, min_z = 1e9;
        float max_x = -1e9, max_y = -1e9, max_z = -1e9;

        for (int idx : indices.indices) {
            PointT p = cloud_obstacles->points[idx];
            cluster->points.push_back(p);
            
            // 更新边界
            min_x = std::min(min_x, p.x);
            min_y = std::min(min_y, p.y);
            min_z = std::min(min_z, p.z);
            max_x = std::max(max_x, p.x);
            max_y = std::max(max_y, p.y);
            max_z = std::max(max_z, p.z);
            
            pcl::PointXYZRGB p_rgb;
            p_rgb.x = p.x;
            p_rgb.y = p.y;
            p_rgb.z = p.z;
            p_rgb.rgb = rgb_float;
            colored_cloud->points.push_back(p_rgb);
        }

        // 计算中心和尺寸
        float center_x = (min_x + max_x) / 2.0f;
        float center_y = (min_y + max_y) / 2.0f;
        float center_z = (min_z + max_z) / 2.0f;
        float size_x = std::max(0.3f, max_x - min_x);
        float size_y = std::max(0.3f, max_y - min_y);
        float size_z = std::max(0.3f, max_z - min_z);

        // 距离计算（到雷达中心的距离）
        float distance = std::sqrt(center_x * center_x + center_y * center_y);

        // 边框 Marker（使用雷达坐标系，跟随小车移动）
        visualization_msgs::Marker bbox;
        bbox.header.frame_id = frame_id;
        bbox.header.stamp = current_time;
        bbox.ns = "cluster_bbox";
        bbox.id = cluster_id;
        bbox.type = visualization_msgs::Marker::CUBE;
        bbox.action = visualization_msgs::Marker::ADD;
        bbox.pose.position.x = center_x;
        bbox.pose.position.y = center_y;
        bbox.pose.position.z = center_z;
        bbox.pose.orientation.w = 1.0;
        bbox.scale.x = size_x;
        bbox.scale.y = size_y;
        bbox.scale.z = size_z;
        bbox.color.r = r / 255.0f;
        bbox.color.g = g / 255.0f;
        bbox.color.b = b / 255.0f;
        bbox.color.a = 0.5f;
        bbox.lifetime = ros::Duration(0.5);  // 增加到 0.5s，防止闪烁
        marker_array.markers.push_back(bbox);

        // 距离文字
        visualization_msgs::Marker text;
        text.header.frame_id = frame_id;
        text.header.stamp = current_time;
        text.ns = "cluster_text";
        text.id = cluster_id;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose.position.x = center_x;
        text.pose.position.y = center_y;
        text.pose.position.z = max_z + 0.5f;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.6;
        text.color.r = 1.0f;
        text.color.g = 1.0f;
        text.color.b = 1.0f;
        text.color.a = 1.0f;
        text.lifetime = ros::Duration(0.5);  // 增加到 0.5s
        
        char dist_str[64];
        snprintf(dist_str, sizeof(dist_str), "ID:%d %.1fm", cluster_id, distance);
        text.text = dist_str;
        marker_array.markers.push_back(text);

        cluster_id++;
    }

    // 发布彩色聚类点云
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*colored_cloud, output);
    output.header.frame_id = frame_id;
    output.header.stamp = current_time;
    pub_clusters.publish(output);

    // 发布 Marker
    if (!marker_array.markers.empty()) {
        pub_markers.publish(marker_array);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_clustering");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 获取参数
    std::string input_topic;
    pnh.param<std::string>("input_topic", input_topic, "/velodyne_points");
    pnh.param<std::string>("frame_id", g_frame_id, "camera_init");

    ROS_INFO("========================================");
    ROS_INFO("实时障碍物聚类节点启动");
    ROS_INFO("订阅话题: %s", input_topic.c_str());
    ROS_INFO("使用坐标系: %s", g_frame_id.c_str());
    ROS_INFO("========================================");

    // 发布器
    pub_clusters = nh.advertise<sensor_msgs::PointCloud2>("/detected_clusters", 1);
    pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/cluster_markers", 1);

    // 订阅激光雷达点云话题
    ros::Subscriber sub = nh.subscribe(input_topic, 1, cloudCallback);

    ROS_INFO("发布话题:");
    ROS_INFO("  - /detected_clusters (彩色聚类点云)");
    ROS_INFO("  - /cluster_markers (边框和距离标注)");
    ROS_INFO("等待点云数据...");

    ros::spin();

    return 0;
}
