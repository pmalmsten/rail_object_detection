#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include "rail_pcl_object_segmentation/pcl_segmenter.hpp"
#include <sstream> 

int main(int argc, char** argv)
{
  int colors[3][8] = { {255, 0, 0, 230, 220, 0, 205, 238}, {0, 255, 0, 230, 20, 199, 198, 154}, {0, 0, 255, 250, 60,
                                                                                                 140, 115, 0}};
  int colorIndex = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (argc >= 2)
  {
    for (int i = 1; i < argc; i++)
    {
      //get pcd
      pcl::PCDReader reader;
      pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
      reader.read(argv[1], *tempCloud);
      *cloud += *tempCloud;
    }

    std::cout << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

    //pass to function	
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
    clouds = *(rail::ExtractObjectClouds<pcl::PointXYZ>(cloud));

    //visulize vector
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    for (int i = 0; i < (int)clouds.size(); i++)
    {
      std::stringstream name;
      name << "cloud" << i;
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(clouds.at(i), colors[0][colorIndex],
                                                                            colors[1][colorIndex],
                                                                            colors[2][colorIndex]);
      viewer->addPointCloud<pcl::PointXYZ>(clouds.at(i), color, name.str());
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name.str());
      colorIndex = (colorIndex + 1) % 8;
    }

    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }
  else
  {
    cout << "Use form ViewSegmentation [filename.pcd]" << endl;
  }

}
