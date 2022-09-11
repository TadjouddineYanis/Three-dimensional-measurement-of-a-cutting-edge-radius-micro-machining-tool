#include<iostream>
#include<vector>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include<X11/Xatom.h>

using namespace std;

vector<double> points;

//Function to program a event point click on the cloud
void evenement_click(const pcl::visualization::PointPickingEvent& event, void* viewer_void){
	float x,y,z;   
  	if (event.getPointIndex () == -1){
    		cout << "pointindex = -1" << endl;
    		return;
  	}
  	event.getPoint(x,y,z);
  	cout << x << "  " << y << " " << z << endl;
  	points.push_back(x);
  	points.push_back(y);
  	points.push_back(z);
}
//Function to get the first point P1 after click on the cloud
pcl::PointXYZ point_P1(int a){
	pcl::PointXYZ P1;
	P1.x = points[(6*a)+0]; 
	P1.y = points[(6*a)+1]; 
	P1.z = points[(6*a)+2];
	return P1;
}
//Function to get the second point P2 after click on the cloud
pcl::PointXYZ point_P2(int a){
	pcl::PointXYZ P2;
	P2.x = points[(6*a)+3]; 
	P2.y = points[(6*a)+4]; 
	P2.z = points[(6*a)+5];
	return P2;
}
//Function to visualize the origin cloud
void visualisation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
  	pcl::visualization::PCLVisualizer viewer ("Initial Cloud");
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 0, 255, 0);
  	viewer.addPointCloud<pcl::PointXYZ> (cloud, cloud_color_handler, "sample cloud");
  	viewer.setBackgroundColor (1, 1, 1);
  	viewer.addCoordinateSystem (1.0);
  	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  	viewer.registerPointPickingCallback(evenement_click, (void*)&viewer);
  	while (!viewer.wasStopped ()){
  		viewer.spinOnce ();
  	}
}
//Function to visualize the origin cloud, the global ROI selected and the line between P1 and P2
void visualisation_cloud_ROI(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr p12, pcl::PointCloud<pcl::PointXYZ>::ConstPtr ROI, pcl::PointCloud<pcl::PointXYZ>::ConstPtr Line){
  	pcl::visualization::PCLVisualizer viewer ("Initial Cloud");
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 0, 255, 0);
  	viewer.addPointCloud<pcl::PointXYZ> (cloud, cloud_color_handler, "Initial_cloud");
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> p12_color_handler (p12, 0, 0, 255);
  	viewer.addPointCloud<pcl::PointXYZ> (p12, p12_color_handler, "p12");
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ROI_color_handler (ROI, 255, 0, 0);
  	viewer.addPointCloud<pcl::PointXYZ> (ROI, ROI_color_handler, "ROI_selected");
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> Line_color_handler (Line, 255, 255, 255);
  	viewer.addPointCloud<pcl::PointXYZ> (Line, Line_color_handler, "LineP1P2");
  	viewer.setBackgroundColor (1, 1, 1);
  	viewer.addCoordinateSystem (1.0);
  	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "Initial_cloud");
  	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "p12");
  	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "ROI_selected");
  	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "LineP1P2");
  	viewer.spin();
}
//Function to visualize the global ROI selected, the cylinder with radius ROI and the estimated cylinder by segmentation
void visualisation_ROI_cylindre(pcl::PointCloud<pcl::PointXYZ>::ConstPtr ROI, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cyl_ROI, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cyl_estimated){
  	pcl::visualization::PCLVisualizer viewer ("Initial Cloud");
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ROI_color_handler (ROI, 255, 255, 255);
  	viewer.addPointCloud<pcl::PointXYZ> (ROI, ROI_color_handler, "ROI_selected");
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cyl_ROI_color_handler (cyl_ROI, 255, 0, 0);
  	viewer.addPointCloud<pcl::PointXYZ>(cyl_ROI, cyl_ROI_color_handler, "cylinder_roi");
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cyl_estimated_color_handler (cyl_estimated, 0, 255, 0);
  	viewer.addPointCloud<pcl::PointXYZ>(cyl_estimated, cyl_estimated_color_handler, "cylinder_estimated");
  	viewer.setBackgroundColor (0, 0, 0);
  	viewer.addCoordinateSystem (1.0);
  	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ROI_selected");
  	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cylinder_roi");
  	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cylinder_estimated");
  	viewer.spin();
}


