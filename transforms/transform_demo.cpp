/*
 * Copyright (c) 2020-2024, GetIoT.tech Team
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-20     luhuadong    the first version
 */

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
    // 检查命令行参数
    if (argc != 2) {
        PCL_ERROR("Usage: %s <input.pcd>\n", argv[0]);
        return -1;
    }

    // 创建点云对象并读取PCD文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read the file %s\n", argv[1]);
        return -1;
    }

    // 创建RT矩阵，将矩阵初始化为单位矩阵
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // 定义旋转矩阵 (绕Z轴旋转45度)
    float theta = M_PI / 4; // 弧度制角度
    transform(0, 0) = cos(theta);
    transform(0, 1) = -sin(theta);
    transform(1, 0) = sin(theta);
    transform(1, 1) = cos(theta);

    // 定义平移向量 (平移 x 方向2.5米, y 方向0米, z 方向1米)
    transform(0, 3) = 2.5;
    transform(1, 3) = 0.0;
    transform(2, 3) = 1.0;

    // 创建变换后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    // 创建可视化对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // 设置原始点云的颜色为白色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> original_color(cloud, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, original_color, "original cloud");

    // 设置变换后点云的颜色为红色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_color(transformed_cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, transformed_color, "transformed cloud");

    // 设置点云大小
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed cloud");

    // 添加坐标系
    viewer->addCoordinateSystem(1.0);
    // viewer->initCameraParameters();

    // 开始可视化
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
