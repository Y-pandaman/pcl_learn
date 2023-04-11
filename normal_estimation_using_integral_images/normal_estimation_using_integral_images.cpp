#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
    
int main(int argc, char ** argv)
{
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile (argv[1], *cloud);  // 读取文件。非有序点云不可用
    
    // 估计的法线
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    // 积分图像方法，估计点云表面法线
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

    // 预测方法：
    // COVARIANCE_MATRIX模式创建9个积分图像，以根据其局部邻域的协方差矩阵为特定点计算法线。
    // AVERAGE_3D_GRADIENT模式创建6个积分图像，以计算3D梯度里面水平和垂直方向的光滑部分，并使用这两个梯度之间的卷积计算法线。
    // AVERAGE_DEPTH_CHANGE模式仅创建单个积分图像，并根据平均深度变化来计算法线。
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX); // 设置深度变化的阀值
    ne.setMaxDepthChangeFactor(0.02f);  // 设置计算法线的区域
    ne.setNormalSmoothingSize(10.0f);   
    ne.setInputCloud(cloud);
    ne.compute(*normals);  // 计算法线

    // 点云+法线 可视化
    pcl::visualization::PCLVisualizer viewer("pcd　viewer");// 显示窗口的名字
    viewer.setBackgroundColor(0.0, 0.0, 0.0);//背景黑色
    //viewer.setBackgroundColor (1, 1, 1);//白色
    viewer.addCoordinateSystem (1.0f, "global");//坐标系
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);
    //pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_color_handler (cloud, 1, 0, 0);//红色
    //viewer.addPointCloud (cloud, cloud_color_handler, "original point cloud")

    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }
    return 0;
}
