#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>
#include <iostream>
#include <pcl/features/vfh.h>
#include <pcl/visualization/pcl_plotter.h>
using namespace std;

int main(int argc, char ** argv){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile (argv[1], *cloud); 

    // ------------创建法线------------------------------
    // 
    // 计算法线，创建法线估计类
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // 创建一个空的kdtree表示，并将其传递给普通的估计对象。
    // 它的内容将根据给定的输入数据集在对象内部填充(因为没有给出其他搜索面)。
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree1); // 设置近邻搜索算法
    // 输出点云 带有法线描述
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.03);//半径内搜索临近点 3cm
    // 计算表面法线特征
    ne.compute(*normals);
    // 
    // ------------创建法线------------------------------


    // 创建VFH估计类，并将输入数据集+法线传递给它
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud);
    vfh.setInputNormals (normals);
    // 如果输入的是带法向量的点云，可以直接输入 vfh.setInputNormals (cloud)

    // 创建一个空的kdtree表示，并将其传递给普通的估计对象。
    // 它的内容将根据给定的输入数据集在对象内部填充(因为没有给出其他搜索面)。
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
    vfh.setSearchMethod (tree2);
    // 输出数据集
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
    // 计算
    vfh.compute (*vfhs);
    std::cout << "vhf feature size : " << vfhs->points.size() << std::endl;  // 应该是1
    

    // 点云+法线 可视化
    pcl::visualization::PCLVisualizer viewer("pcd　viewer");// 显示窗口的名字
    viewer.setBackgroundColor(0.0, 0.0, 0.0);//背景黑色
    viewer.addCoordinateSystem (1.0f, "global");//坐标系 
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");//渲染属性，可视化工具，3维数据
    // 参数5表示整个点云中每5各点显示一个法向量（若全部显示，可设置为1，  15表示法向量的长度，最后一个参数暂时还不知道 如何影响的）
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 5, 15, "normal");
   
    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));   //随时间
    }

    // ========直方图可视化=============================
    // 运行时出现错误：vtkOpenGLTexture (0000000002540430): No scalar values found for texture input!
    // 原因：创建vtkTextActor时，未调用SetInput()方法；或调用SetInput()方法，参数给的空字符串。
    // 这都相当于创建了一个空vtkTextActor，导致该错误发生。
    // 解决方法：创建vtkTextActor时进行初始化，即调用SetInput()方法，且必须给出参数，哪怕是一个空格字符创。
    // pcl::visualization::PCLHistogramVisualizer view;//直方图可视化
    // view.setBackgroundColor(255,0,0);//背景
    // view.addFeatureHistogram<pcl::PFHSignature125> (*pfhs,"pfh",100); 
    // view.spinOnce();  //循环的次数
    //view.spin();  //无限循环

    // 替代方法
    pcl::visualization::PCLPlotter plotter;
    plotter.addFeatureHistogram(*vfhs, 300); //设置的横坐标长度，该值越大，则显示的越细致
    plotter.plot();
    
    return 0;
}