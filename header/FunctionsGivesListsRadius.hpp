#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double, double);
pcl::PointCloud<pcl::PointXYZ>::Ptr addNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string noiseType, double);
pcl::ModelCoefficients::Ptr EstimatePerpendicularPlaneFrom2Points(pcl::PointXYZ P1, pcl::PointXYZ P2);
double positionPointPlane(pcl::PointXYZ &P, pcl::ModelCoefficients::Ptr coefficients);
pcl::PointCloud<pcl::PointXYZ>::Ptr Global_ROI(pcl::PointXYZ P1, pcl::PointXYZ P2, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr Line_P1_P2(pcl::PointXYZ P1, pcl::PointXYZ P2);
pcl::PointCloud<pcl::PointXYZ>::Ptr generateCircleData(const double, const double, const double, const double);
double DistancePointToLine(pcl::PointXYZ  A, pcl::PointXYZ B, pcl::PointXYZ F);
pcl::PointCloud<pcl::PointXYZ>::Ptr area_interest_line(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double, pcl::PointXYZ P1, pcl::PointXYZ P2);
pcl::ModelCoefficients::Ptr radius_estimated(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, float, float);
void estimate_all_radius(pcl::PointCloud<pcl::PointXYZ>::Ptr ROI, pcl::PointCloud<pcl::PointXYZ>::ConstPtr points_drt, pcl::PointXYZ P1, pcl::PointXYZ P2, float, float, int, float, float);
std::vector<double> ROI_values(void);
std::vector<double> estimated_radius_values(void);
void write_file(std::vector<double> x, std::vector<double> y, std::string path, std::string name_roi, std::string name_radius_estim);
double distance_P1_P2(pcl::PointXYZ P1, pcl::PointXYZ P2);

