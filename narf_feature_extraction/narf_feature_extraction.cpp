#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>   // 深度图像
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>   // narf关键点检测
#include <pcl/features/narf_descriptor.h>  // narf特征
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointType;

// 参数
float angular_resolution = 0.5f; //角坐标分辨率
float support_size = 0.2f;  //感兴趣点的尺寸（球面的直径）
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;  //坐标框架：相机框架（而不是激光框架）
bool setUnseenToMaxRange = false; //是否将所有不可见的点 看作 最大距离
bool rotation_invariant = true;

// 帮助信息
void printUsage (const char* progName){
  std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
            << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
            << "-m           Treat all unseen points to max range\n"
            << "-s <float>   support size for the interest points (diameter of the used sphere - "
                                                                  "default "<<support_size<<")\n"
            << "-o <0/1>     switch rotational invariant version of the feature on/off"
            <<               " (default "<< (int)rotation_invariant<<")\n"
            << "-h           this help\n"
            << "\n\n";
}

void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose){
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}

int main (int argc, char** argv)
{

  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  if (pcl::console::find_argument (argc, argv, "-m") >= 0)
  {
    setUnseenToMaxRange = true;  //将所有不可见的点 看作 最大距离
    cout << "Setting unseen values in range image to maximum range readings.\n";
  }
  if (pcl::console::parse (argc, argv, "-o", rotation_invariant) >= 0)
    cout << "Switching rotation invariant feature version "<< (rotation_invariant ? "on" : "off")<<".\n";
  int tmp_coordinate_frame;  //坐标框架：相机框架（而不是激光框架）
  if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
  {
    coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
    cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
  }
  // 感兴趣点的尺寸（球面的直径）
  if (pcl::console::parse (argc, argv, "-s", support_size) >= 0)
    cout << "Setting support size to "<<support_size<<".\n";
  // 角坐标分辨率
  if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
    cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
  angular_resolution = pcl::deg2rad (angular_resolution);
  
  // 读取点云文件或者创建一个点云文件（如果没有）
  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>); //点云对象指针
  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr; //引用　上面点云的别名　常亮指针
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges; //带视角的点云
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ()); //仿射变换
  //检查参数中是否有pcd格式文件名，返回参数向量中的索引号
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  if (!pcd_filename_indices.empty ())
  {
    std::string filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile (filename, point_cloud) == -1) //如果指定了pcd文件，读取pcd文件
    {
      cerr << "Was not able to open file \""<<filename<<"\".\n";
      printUsage (argv[0]);
      return 0;
    }
    //设置传感器的姿势
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);
    //读取远距离文件?
    std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
    if (pcl::io::loadPCDFile (far_ranges_filename.c_str (), far_ranges) == -1)
      std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
  }
  else  //没有指定pcd文件，生成点云，并填充它
  {
    setUnseenToMaxRange = true;  //将所有不可见的点 看作 最大距离
    cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
    for (float x=-0.5f; x<=0.5f; x+=0.01f)
    {
      for (float y=-0.5f; y<=0.5f; y+=0.01f)
      {
        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
        point_cloud.points.push_back (point);  //设置点云中点的坐标
      }
    }
    point_cloud.width = (int) point_cloud.points.size ();  point_cloud.height = 1;
  }
  
  // 从点云中创建深度图像RangeImage
  // 直接把三维的点云投射成二维的图像
  float noise_level = 0.0;  // noise level表示的是容差率，因为1°X1°的空间内很可能不止一个点，
                            // noise level = 0则表示去最近点的距离作为像素值，如果=0.05则表示在最近点及其后5cm范围内求个平均距离
  
  float min_range = 0.0f;  // minRange表示深度最小值，如果=0则表示取1°X1°的空间内最远点，近的都忽略
  int border_size = 1;  //bordersieze表示图像周边点 
  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);  //创建RangeImage对象（智能指针）
  pcl::RangeImage& range_image = *range_image_ptr;     //RangeImage的引用
  //  从点云创建深度图像
  //  rangeImage也是PCL的基本数据结构
  //  pcl::RangeImage rangeImage;
  //  球坐标系
  //  角分辨率
  //  float angularResolution = (float) (1.0f * (M_PI/180.0f));  //   1.0 degree in radians　弧度
  //  phi可以取360°
  //  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  //  a取180°
  //  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
  //  半圆扫一圈就是整个图像了

  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  range_image.integrateFarRanges (far_ranges);  // 整合远距离点云
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();
  
  // 可视化
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (1, 1, 1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
  viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");  // 添加点云
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
  //viewer.addCoordinateSystem (1.0f, "global");
  //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
  //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  viewer.initCameraParameters ();  // 初始化相机参数
  setViewerPose (viewer, range_image.getTransformationToWorldSystem ());
  
  // 展示深度图像(平面图)
  pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  range_image_widget.showRangeImage (range_image);
  
  // 提取NARF关键点
  pcl::RangeImageBorderExtractor range_image_border_extractor; //创建深度图像的边界提取器，用于提取NARF关键点
  pcl::NarfKeypoint narf_keypoint_detector; 
  narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor); //创建NARF对象
  narf_keypoint_detector.setRangeImage (&range_image);  //设置点云对应的深度图
  narf_keypoint_detector.getParameters ().support_size = support_size;  // 感兴趣点的尺寸（球面的直径）
  
  pcl::PointCloud<int> keypoint_indices;  //用于存储关键点的索引 PointCloud<int>
  narf_keypoint_detector.compute (keypoint_indices);  //计算NARF关键点
  std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";

  // 在深度图像小部件中显示关键点
  // for (size_t i=0; i<keypoint_indices.points.size (); ++i)
  //   range_image_widget.markPoint (keypoint_indices.points[i]%range_image.width,
  //                                 keypoint_indices.points[i]/range_image.width);
  
  // 在3D视图中显示关键点
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>); //创建关键点指针
  pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr; //引用
  keypoints.points.resize (keypoint_indices.points.size ()); //初始化大小
  for (size_t i=0; i<keypoint_indices.points.size (); ++i)  //按照索引获得关键点
    keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_ptr, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints"); //添加显示关键点
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  //渲染属性，可视化工具，3维数据， 其中PCL_VISUALIZER_POINT_SIZE表示设置点的大小为7
  
  // 提取NARF描述子
  std::vector<int> keypoint_indices2; //用于存储关键点的索引 vector<int>  
  // 将索引复制到用作特征输入的向量
  keypoint_indices2.resize (keypoint_indices.points.size ());
  for (unsigned int i=0; i<keypoint_indices.size (); ++i) // 这一步是获得正确的向量类型所必需的
    keypoint_indices2[i]=keypoint_indices.points[i];  //narf关键点 索引
  pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices2);  //narf特征描述子
  narf_descriptor.getParameters ().support_size = support_size;
  narf_descriptor.getParameters ().rotation_invariant = rotation_invariant;
  // 生成的PointCloud包含类型Narf36,并将描述符存储为36个元素float和x，y，z，roll，
  // pitch，yaw，以描述该功能所在的局部坐标系被提取。现在可以将描述符与曼哈顿距离（绝对差之和）进行比较。
  pcl::PointCloud<pcl::Narf36> narf_descriptors;  
  narf_descriptor.compute (narf_descriptors);
  std::cout << "Extracted "<<narf_descriptors.size ()<<" descriptors for "
                      <<keypoint_indices.points.size ()<< " keypoints.\n";
  
  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer.wasStopped ())
  {
    range_image_widget.spinOnce ();  // process GUI events
    viewer.spinOnce ();
    pcl_sleep(0.01);
  }
}
