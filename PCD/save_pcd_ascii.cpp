/*
 * Copyright (c) 2020-2024, GetIoT.tech Team
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-27     luhuadong    the first version
 */

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

int main()
{
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // 随机生成点云数据
    srand(static_cast<unsigned int>(time(NULL)));
    for (int i = 0; i < 1000; ++i)
    {
        pcl::PointXYZI point;
        point.x = 10.0f * rand() / RAND_MAX;  // 在 x 方向上随机生成坐标
        point.y = 10.0f * rand() / RAND_MAX;  // 在 y 方向上随机生成坐标
        point.z = 10.0f * rand() / RAND_MAX;  // 在 z 方向上随机生成坐标
        point.intensity = 255.0f * rand() / RAND_MAX;  // 生成随机强度值
        cloud->push_back(point);
    }

    // 保存点云数据到 PCD 文件
    pcl::io::savePCDFileASCII("output.pcd", *cloud);

    std::cout << "Saved " << cloud->points.size() << " data points to output.pcd." << std::endl;

    return 0;
}
