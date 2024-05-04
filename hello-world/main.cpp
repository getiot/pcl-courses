#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>

int main()
{
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // 随机生成点云数据
    srand(time(NULL));
    for (int i = 0; i < 1000; ++i)
    {
        pcl::PointXYZI point;
        point.x = 10.0f * rand() / RAND_MAX;  // 在 x 方向上随机生成坐标
        point.y = 10.0f * rand() / RAND_MAX;  // 在 y 方向上随机生成坐标
        point.z = 10.0f * rand() / RAND_MAX;  // 在 z 方向上随机生成坐标
        point.intensity = rand() % 256;       // 生成随机强度值
        cloud->push_back(point);
    }

    // 创建可视化窗口对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

    // 设置点云颜色为灰色
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "cloud");
    // 设置点云颜色为 intensity 值
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(cloud, "intensity");
    
    // 将点云添加到可视化窗口中
    viewer->addPointCloud<pcl::PointXYZI>(cloud, fildColor, "cloud");

    // 设置点云大小
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    // 显示可视化窗口，直到用户关闭窗口为止
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(1000);  // 每隔100毫秒渲染一次
    }

    return 0;
}
