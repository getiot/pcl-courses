/*
 * Copyright (c) 2020-2024, GetIoT.tech Team
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-27     luhuadong    the first version
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// using namespace pcl::visualization;

int main(int argc, char** argv)
{
    // 检查命令行参数
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <path-to-pcd-file>" << std::endl;
        return -1;
    }

    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // 从文件中读取点云数据
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", argv[1]);
        return -1;
    }

    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << argv[1] << std::endl;

    // 创建可视化窗口对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCD Viewer"));

    // 创建颜色处理对象
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(cloud, "intensity");

    // 将点云添加到可视化窗口中
    if (fildColor.isCapable()) {
        // 如果点云包含 Intensity 字段，则使用 Intensity 渲染点云颜色
        viewer->addPointCloud<pcl::PointXYZI>(cloud, fildColor, "sample cloud");
    }
    else {
        // 如果没有 Intensity 字段，则使用默认颜色渲染
        viewer->addPointCloud<pcl::PointXYZI>(cloud, "sample cloud");
    }

    // 设置点云大小
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

    // 显示可视化窗口，直到用户关闭窗口为止
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);  // 每隔100毫秒渲染一次
    }

    return 0;
}
