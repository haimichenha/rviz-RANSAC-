/**
 * @file lidar_perception_node.cpp
 * @brief 激光雷达点云感知节点 - 实验3
 * 
 * 功能：
 * 1. VoxelGrid 下采样 - 减少点云数量，提高处理效率
 * 2. 地面分割 (RANSAC) - 使用随机采样一致性算法分割地面
 * 3. 欧几里德聚类 - 将非地面点聚类为独立障碍物
 * 4. 可视化 - 发布 Marker 显示边界框和距离
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// 滤波器
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

// 地面分割
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

// 聚类
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

#include <vector>
#include <cmath>

// 类型别名
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class LidarPerceptionNode
{
public:
    LidarPerceptionNode() : nh_("~")
    {
        // 参数设置 (可通过 launch 文件配置)
        nh_.param<std::string>("input_topic", input_topic_, "/velodyne_points");
        nh_.param<double>("voxel_leaf_size", voxel_leaf_size_, 0.1);  // 下采样体素大小
        nh_.param<double>("ground_threshold", ground_threshold_, 0.3); // 地面分割阈值
        nh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.5); // 聚类距离阈值
        nh_.param<int>("min_cluster_size", min_cluster_size_, 50);       // 最小聚类点数
        nh_.param<int>("max_cluster_size", max_cluster_size_, 25000);    // 最大聚类点数
        
        // 预处理参数
        nh_.param<double>("min_x", min_x_, -50.0);
        nh_.param<double>("max_x", max_x_, 50.0);
        nh_.param<double>("min_y", min_y_, -50.0);
        nh_.param<double>("max_y", max_y_, 50.0);
        nh_.param<double>("min_z", min_z_, -2.0);
        nh_.param<double>("max_z", max_z_, 3.0);

        // 订阅与发布
        sub_pointcloud_ = nh_.subscribe(input_topic_, 1, &LidarPerceptionNode::pointCloudCallback, this);
        
        pub_downsampled_ = nh_.advertise<sensor_msgs::PointCloud2>("downsampled", 1);
        pub_ground_ = nh_.advertise<sensor_msgs::PointCloud2>("ground", 1);
        pub_obstacles_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacles", 1);
        pub_clusters_ = nh_.advertise<sensor_msgs::PointCloud2>("clusters", 1);
        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 1);

        ROS_INFO("LiDAR Perception Node initialized.");
        ROS_INFO("  Input topic: %s", input_topic_.c_str());
        ROS_INFO("  Voxel size: %.2f m", voxel_leaf_size_);
        ROS_INFO("  Ground threshold: %.2f m", ground_threshold_);
        ROS_INFO("  Cluster tolerance: %.2f m", cluster_tolerance_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_pointcloud_;
    ros::Publisher pub_downsampled_;
    ros::Publisher pub_ground_;
    ros::Publisher pub_obstacles_;
    ros::Publisher pub_clusters_;
    ros::Publisher pub_markers_;

    // 参数
    std::string input_topic_;
    double voxel_leaf_size_;
    double ground_threshold_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;

    /**
     * @brief 点云回调函数 - 主处理流程
     */
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // 转换为 PCL 格式
        PointCloudT::Ptr cloud_input(new PointCloudT);
        pcl::fromROSMsg(*msg, *cloud_input);

        if (cloud_input->empty())
        {
            ROS_WARN("Received empty point cloud!");
            return;
        }

        std::string frame_id = msg->header.frame_id;
        ros::Time stamp = msg->header.stamp;

        // ============ 步骤 1: 预处理 (ROI 过滤) ============
        PointCloudT::Ptr cloud_filtered(new PointCloudT);
        passThroughFilter(cloud_input, cloud_filtered);

        // ============ 步骤 2: 体素下采样 ============
        PointCloudT::Ptr cloud_downsampled(new PointCloudT);
        voxelGridFilter(cloud_filtered, cloud_downsampled);
        publishPointCloud(pub_downsampled_, cloud_downsampled, frame_id, stamp);

        ROS_DEBUG("Downsampled: %zu -> %zu points", cloud_input->size(), cloud_downsampled->size());

        // ============ 步骤 3: RANSAC 地面分割 ============
        PointCloudT::Ptr cloud_ground(new PointCloudT);
        PointCloudT::Ptr cloud_obstacles(new PointCloudT);
        groundSegmentation(cloud_downsampled, cloud_ground, cloud_obstacles);

        publishPointCloud(pub_ground_, cloud_ground, frame_id, stamp);
        publishPointCloud(pub_obstacles_, cloud_obstacles, frame_id, stamp);

        ROS_DEBUG("Ground: %zu, Obstacles: %zu", cloud_ground->size(), cloud_obstacles->size());

        // ============ 步骤 4: 欧几里德聚类 ============
        std::vector<pcl::PointIndices> cluster_indices;
        euclideanClustering(cloud_obstacles, cluster_indices);

        // 处理聚类结果并可视化
        PointCloudT::Ptr cloud_clustered(new PointCloudT);
        visualization_msgs::MarkerArray marker_array;
        
        processClusters(cloud_obstacles, cluster_indices, cloud_clustered, marker_array, frame_id, stamp);

        publishPointCloud(pub_clusters_, cloud_clustered, frame_id, stamp);
        pub_markers_.publish(marker_array);

        ROS_INFO_THROTTLE(1.0, "Detected %zu clusters", cluster_indices.size());
    }

    /**
     * @brief 直通滤波 - ROI 区域过滤
     */
    void passThroughFilter(const PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_out)
    {
        pcl::PassThrough<PointT> pass;
        PointCloudT::Ptr temp(new PointCloudT);

        // X 方向过滤
        pass.setInputCloud(cloud_in);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(min_x_, max_x_);
        pass.filter(*temp);

        // Y 方向过滤
        pass.setInputCloud(temp);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(min_y_, max_y_);
        pass.filter(*temp);

        // Z 方向过滤
        pass.setInputCloud(temp);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_z_, max_z_);
        pass.filter(*cloud_out);
    }

    /**
     * @brief 体素网格下采样
     */
    void voxelGridFilter(const PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_out)
    {
        pcl::VoxelGrid<PointT> voxel;
        voxel.setInputCloud(cloud_in);
        voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel.filter(*cloud_out);
    }

    /**
     * @brief RANSAC 地面分割
     * 使用平面模型拟合: ax + by + cz + d = 0
     */
    void groundSegmentation(const PointCloudT::Ptr& cloud_in,
                            PointCloudT::Ptr& cloud_ground,
                            PointCloudT::Ptr& cloud_obstacles)
    {
        if (cloud_in->size() < 10)
        {
            *cloud_obstacles = *cloud_in;
            return;
        }

        // RANSAC 平面分割器
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(ground_threshold_);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        seg.setInputCloud(cloud_in);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            ROS_WARN("Could not estimate a planar model for the ground.");
            *cloud_obstacles = *cloud_in;
            return;
        }

        // 提取地面点
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_ground);

        // 提取非地面点 (障碍物)
        extract.setNegative(true);
        extract.filter(*cloud_obstacles);

        // 输出平面方程参数
        ROS_DEBUG("Ground plane: %.3fx + %.3fy + %.3fz + %.3f = 0",
                  coefficients->values[0], coefficients->values[1],
                  coefficients->values[2], coefficients->values[3]);
    }

    /**
     * @brief 欧几里德聚类
     */
    void euclideanClustering(const PointCloudT::Ptr& cloud_in,
                             std::vector<pcl::PointIndices>& cluster_indices)
    {
        if (cloud_in->empty())
            return;

        // 创建 KdTree 用于加速搜索
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud_in);

        // 欧几里德聚类提取器
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_in);
        ec.extract(cluster_indices);
    }

    /**
     * @brief 处理聚类结果，生成着色点云和 Marker
     */
    void processClusters(const PointCloudT::Ptr& cloud_obstacles,
                         const std::vector<pcl::PointIndices>& cluster_indices,
                         PointCloudT::Ptr& cloud_clustered,
                         visualization_msgs::MarkerArray& marker_array,
                         const std::string& frame_id,
                         const ros::Time& stamp)
    {
        // 预定义颜色 (RGB)
        std::vector<std::tuple<float, float, float>> colors = {
            {1.0, 0.0, 0.0},  // 红
            {0.0, 1.0, 0.0},  // 绿
            {0.0, 0.0, 1.0},  // 蓝
            {1.0, 1.0, 0.0},  // 黄
            {1.0, 0.0, 1.0},  // 紫
            {0.0, 1.0, 1.0},  // 青
            {1.0, 0.5, 0.0},  // 橙
            {0.5, 0.0, 1.0},  // 紫罗兰
        };

        int cluster_id = 0;
        
        for (const auto& indices : cluster_indices)
        {
            // 计算边界框
            float min_x = std::numeric_limits<float>::max();
            float min_y = std::numeric_limits<float>::max();
            float min_z = std::numeric_limits<float>::max();
            float max_x = std::numeric_limits<float>::lowest();
            float max_y = std::numeric_limits<float>::lowest();
            float max_z = std::numeric_limits<float>::lowest();

            float sum_x = 0, sum_y = 0, sum_z = 0;

            for (int idx : indices.indices)
            {
                const PointT& pt = cloud_obstacles->points[idx];
                cloud_clustered->points.push_back(pt);

                min_x = std::min(min_x, pt.x);
                min_y = std::min(min_y, pt.y);
                min_z = std::min(min_z, pt.z);
                max_x = std::max(max_x, pt.x);
                max_y = std::max(max_y, pt.y);
                max_z = std::max(max_z, pt.z);

                sum_x += pt.x;
                sum_y += pt.y;
                sum_z += pt.z;
            }

            // 计算质心
            float centroid_x = sum_x / indices.indices.size();
            float centroid_y = sum_y / indices.indices.size();
            float centroid_z = sum_z / indices.indices.size();

            // 计算到原点的距离
            float distance = std::sqrt(centroid_x * centroid_x + 
                                       centroid_y * centroid_y + 
                                       centroid_z * centroid_z);

            // 获取颜色
            auto& color = colors[cluster_id % colors.size()];

            // ============ 创建边界框 Marker ============
            visualization_msgs::Marker box_marker;
            box_marker.header.frame_id = frame_id;
            box_marker.header.stamp = stamp;
            box_marker.ns = "bounding_boxes";
            box_marker.id = cluster_id;
            box_marker.type = visualization_msgs::Marker::CUBE;
            box_marker.action = visualization_msgs::Marker::ADD;

            box_marker.pose.position.x = (min_x + max_x) / 2.0;
            box_marker.pose.position.y = (min_y + max_y) / 2.0;
            box_marker.pose.position.z = (min_z + max_z) / 2.0;
            box_marker.pose.orientation.w = 1.0;

            box_marker.scale.x = max_x - min_x;
            box_marker.scale.y = max_y - min_y;
            box_marker.scale.z = max_z - min_z;

            box_marker.color.r = std::get<0>(color);
            box_marker.color.g = std::get<1>(color);
            box_marker.color.b = std::get<2>(color);
            box_marker.color.a = 0.3;

            box_marker.lifetime = ros::Duration(0.1);

            marker_array.markers.push_back(box_marker);

            // ============ 创建文字标签 (距离) ============
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = frame_id;
            text_marker.header.stamp = stamp;
            text_marker.ns = "distances";
            text_marker.id = cluster_id;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;

            text_marker.pose.position.x = centroid_x;
            text_marker.pose.position.y = centroid_y;
            text_marker.pose.position.z = max_z + 0.5;  // 文字在顶部
            text_marker.pose.orientation.w = 1.0;

            text_marker.scale.z = 0.5;  // 文字高度

            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%.1f m", distance);
            text_marker.text = buffer;

            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;

            text_marker.lifetime = ros::Duration(0.1);

            marker_array.markers.push_back(text_marker);

            cluster_id++;
        }

        cloud_clustered->width = cloud_clustered->points.size();
        cloud_clustered->height = 1;
        cloud_clustered->is_dense = true;
    }

    /**
     * @brief 发布点云消息
     */
    void publishPointCloud(ros::Publisher& pub,
                           const PointCloudT::Ptr& cloud,
                           const std::string& frame_id,
                           const ros::Time& stamp)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.frame_id = frame_id;
        msg.header.stamp = stamp;
        pub.publish(msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_perception_node");
    
    LidarPerceptionNode node;
    
    ros::spin();
    
    return 0;
}
