#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/visualization/histogram_visualizer.h> //直方图的可视化
#include <pcl/visualization/pcl_plotter.h>// 直方图的可视化 方法2

int main(int argc, char const *argv[]){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // 读取文件,必须是带法线的结构点云数据
    // pcl::PCDReader reader;
    // reader.read( "../../data/table_scene_lms400.pcd", *cloud); // 非有序点云不可用
    // if(cloud==NULL) { 
    //     cout << "pcd file read err" << endl; return -1;
    // }
    // cout << "PointCLoud size() " << cloud->width * cloud->height << " data points :" << pcl::getFieldsList (*cloud) << "." << endl;

    pcl::io::loadPCDFile (argv[1], *cloud);  // 读取文件。非有序点云不可用

    // 计算法线，创建法线估计类
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // ------------创建法线------------------------------
    // 创建一个空的kdtree表示，并将其传递给普通的估计对象。
    // 它的内容将根据给定的输入数据集在对象内部填充(因为没有给出其他搜索面)。
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree); // 设置近邻搜索算法
    // 输出点云 带有法线描述
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.03);//半价内搜索临近点 3cm
    // 计算表面法线特征
    ne.compute(*cloud_normals);
    // ------------创建法线------------------------------

    // 创建PFH估计类，并将输入数据集+法线传递给它
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud (cloud);
    pfh.setInputNormals (cloud_normals);
    // 如果输入的是带法向量的点云，可以直接执行 pfh.setInputNormals(cloud)

    //创建一个空的kd树表示法，并把它传递给PFH估计对象。
    //基于已给的输入数据集，建立kdtree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
    pfh.setSearchMethod (tree2);//设置近邻搜索算法

    // 输出数据集
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());

    // 对于每一个点都用半径为5cm的近邻搜索方式
    // 重要:这里使用的半径必须大于用于估计表面法线的半径!!
    pfh.setRadiusSearch (0.05);
    // 计算pfh特征值
    pfh.compute (*pfhs);

    std::cout << "phf feature size : " << pfhs->points.size() << std::endl;
    // 应该与input cloud->points.size ()有相同的大小，即每个点都有一个pfh特征向量

    // 点云+法线 可视化
    pcl::visualization::PCLVisualizer viewer("pcd　viewer");// 显示窗口的名字
    viewer.setBackgroundColor(0.0, 0.0, 0.0);//背景黑色
    viewer.addCoordinateSystem (1.0f, "global");//坐标系
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals);

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
    plotter.addFeatureHistogram(*pfhs, 300); //设置的横坐标长度，该值越大，则显示的越细致
    plotter.plot();

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }

    return 0;
}

