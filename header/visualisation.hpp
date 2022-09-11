#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
void evenement_click(const pcl::visualization::PointPickingEvent& event, void* viewer_void);
pcl::PointXYZ point_P1(int a);
pcl::PointXYZ point_P2(int a);
void visualisation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
void visualisation_cloud_ROI(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr p12, pcl::PointCloud<pcl::PointXYZ>::ConstPtr ROI, pcl::PointCloud<pcl::PointXYZ>::ConstPtr Line);
void visualisation_ROI_cylindre(pcl::PointCloud<pcl::PointXYZ>::ConstPtr ROI, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cyl_ROI, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cyl_estimated);
