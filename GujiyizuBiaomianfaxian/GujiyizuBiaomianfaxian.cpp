#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char const *argv[])
{
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1){
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    // 创建正常的估计类，并将输入数据集传递给它
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // 创建一个空的kdtree表示，并将其传递给普通的估计对象。
    // 它的内容将根据给定的输入数据集在对象内部填充(因为没有给出其他搜索面)。
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // 输出数据集
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // 对于每一个点都用半径为3cm的近邻搜索方式
    ne.setRadiusSearch (0.03);

    // 计算法线
    ne.compute (*cloud_normals);

    // cloud_normals->size () should have the same size as the input cloud->size ()
    std::cout << cloud->size() << "  " << cloud_normals->size() << std::endl;

    //可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("为输入数据集中的所有点估计一组表面法线"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud,"cloud");while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
    }

    return 0;
}
