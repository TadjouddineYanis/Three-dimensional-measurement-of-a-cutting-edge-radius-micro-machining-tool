#include<iostream>
#include<string>
#include <cmath>
#include <pcl/io/pcd_io.h>

#include"FunctionsGivesListsRadius.hpp"
#include"FunctionsGivesRadiusTool.hpp"
#include"visualisation.hpp"
#include "figures.hpp"

/**** Example of solution to measure the edge radius of a micro usinage tool ****/
// To execute, please choose the .pcd file, the right ratio and the treshold and then we can lunch

#define ratioo 1 //0.02773 for outilneuf, 0.02791 for outiluse, 0.1146 the others
#define percentt 100
#define seuil 0.1
#define iteration 1000
#define noise 0 //put 1, if we want to add some noise on the point cloud
#define noiseamplitude 0.2
#define LOOP 1 //Put 1 if we want to execute a single time, and x value to execute x time, all the algorithms.
#define numberStep 100 //maximum value for the loop, to compute the radius ROI.
#define pas 0.1 //step between the ROI values.
#define display 0 //put 1 if we want to see the cylinders.
#define point 15 //check linearity, if we have, for example, 15 points, who those are the same value of finit difference.
#define epsilon 0.5 //to test equality, will be very small.
#define seuil_1 15 //seuil, when we don't detect a evident linear part.
#define seuil_2 5 //seuil, when we have detected a evident linear part.
#define seuil_r 0.01 //seuil line fitting with LMeDs robest.

using namespace std;

//specify the path who we can find the ply or pcd file. And the path wo we can save the radius of ROI and the one estimated found.
string path="/home/tadjouddine/Programmes_c++/Programmes_Internship/Final_Code_Radius_Measurement/nuages/";
string file="simuOutils.pcd"; //"simuOutils.pcd", "n1densepair31-32.pcd", "n2densepair31-32.pcd", "n3-1k-densepair31-32.pcd", "u31kdensepair30-31.pcd", "outilsuse.pcd", "outilsneuf.pcd"
string path_radius="/home/tadjouddine/Programmes_c++/Programmes_Internship/Final_Code_Radius_Measurement/radius_measured/";
string noisetype = "bruitNormal";//"bruitBlanc", "bruitNormal", "bruitSinusoidal"and "bruitAberrant"
string name_figure = "test";

int main(){
	pcl::PointXYZ P1, P2;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ROI (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr p1_p2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Line_p1_p2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	vector<double> roi, radius_estim, x_filtre, y_filtre, list_radius, list_distance; short linearity; double distance, edge_radius;
	int bleu[3] = {0, 0, 1}; int rouge[3] = {1, 0, 0}; int vert[3] = {0, 1, 0}; int t(0);
	vector<double> diff_finie, extract_points_x, extract_points_y, robest, inter, xi(1,0), yi(1,0);
	
	while(t<LOOP){
		reader.read(path+file, *cloud);
		cout << "InitialPointCloud has: " << cloud->size() << " data points." << endl;
		cloud = filterCloud(cloud, ratioo, percentt);
		cout << "FilteredPointCloud has: " << cloud->size() << " data points." << endl;
		if(noise == 1){
			cloud = addNoise(cloud, noisetype, noiseamplitude);
			cout << "NoisePointCloud has: " << cloud->size() << " data points." << endl;
		}
		visualisation(cloud);
		P1 = point_P1(t);
		cout << "P1: " << P1.x << "   " << P1.y << "   " << P1.z << endl;
		P2 = point_P2(t);
		cout << "P2: " << P2.x << "   " << P2.y << "   " << P2.z << endl;
		p1_p2 -> points.push_back(P1); p1_p2 -> points.push_back(P2);
		ROI = Global_ROI(P1, P2, cloud);
		cout << "ROI PointCloud has: " << ROI->size() << " data points." << endl;
		p1_p2 -> points.push_back(P1); p1_p2 -> points.push_back(P2);
		Line_p1_p2 = Line_P1_P2(P1, P2);
		visualisation_cloud_ROI(cloud, p1_p2, ROI, Line_p1_p2);
		estimate_all_radius(ROI, Line_p1_p2, P1, P2, numberStep, pas, display, seuil, iteration);
		roi = ROI_values();
		radius_estim = estimated_radius_values();
		//cout << "roi: " << endl;
		//print_vector(roi);
		//cout << "radius estimated: " << endl;
		//print_vector(radius_estim);
		x_filtre.clear();
		y_filtre.clear();
		x_filtre.push_back(roi[0]);
		y_filtre.push_back(radius_estim[0]);
		for(int q=0;q<roi.size()-1;q++){
			double dist;
			dist = sqrt(pow(roi[q+1] - roi[q], 2) + pow(radius_estim[q+1] - radius_estim[q], 2));
			//cout << "distance: " << dist << endl;
			if(dist < 2.0){
				x_filtre.push_back(roi[q+1]);
				y_filtre.push_back(radius_estim[q+1]);
			}
		}
		//cout << "x_filtre: " << endl;
		//print_vector(x_filtre);
		//cout << "y_filtre: " << endl;
		//print_vector(y_filtre);
		plot(roi, radius_estim, bleu, L"Graph of data ROI and Radius estimated", L"ROI radius(micro metre)", L"Edge radius(micro metre)", "Data Graph");
		plot(x_filtre, y_filtre, bleu, L"Graph of data ROI and Radius estimated", L"ROI radius(micro metre)", L"Edge radius(micro metre)", "Data Graph filtered");
		diff_finie = finite_difference(x_filtre, y_filtre);
		//cout << "difference_finie: " << endl;
		//print_vector(diff_finie);
		linearity = check_linear_part(diff_finie, point, epsilon);
		cout << "linearity = " << linearity << endl;
		extract_points_x = extract_points_linear_x(diff_finie, x_filtre, linearity, seuil_1, seuil_2, numberStep);
		extract_points_y = extract_points_linear_y(diff_finie, y_filtre, linearity, seuil_1, seuil_2, numberStep);
		//print_vector(extract_points_x);
		//print_vector(extract_points_y);
		plot(extract_points_x, extract_points_y, rouge, L"Graph of data selectionned", L"ROI radius(micro metre)", L"Edge radius(miro metre)", "Points_extracted");
		robest = robest_line_fitting(extract_points_x, extract_points_y, seuil_r);
		cout << "Robest: " << robest[0] <<" *x + " << robest[1] << endl;
		edge_radius = robest[1]; //intersection between the line obtained with robest and the axis Y
		cout << "edge_radius: " << edge_radius << endl;
		xi[0] = 0;
		yi[0] = edge_radius;
		distance = distance_P1_P2(P1, P2);
		list_radius.push_back(edge_radius);
		list_distance.push_back(distance);
		cout << "Loop: " << t << endl;
		t++;		
		//Write some file on the disk to plot also with python
		write_file(roi, radius_estim, path_radius, "roi", "radius");
		write_file(x_filtre, y_filtre, path_radius, "roi_filtered", "radius_filtered");
		write_file(extract_points_x, extract_points_y, path_radius, "roi_extract", "radius_extract");
		write_file(equation_droite(robest, extract_points_x), radius_equation(extract_points_x, (72*M_PI)/180), path_radius, "robest_line", "radius_equation");		
		write_file(xi, yi, path_radius, "inter_x", "inter_y");
	}
	write_file(list_distance, list_radius, path_radius, "distance", "edge_radius");
	plot(list_distance, list_radius, bleu, L"Radius Tool according to the points P1 and P2", L"Distance between P1 and P2(micro metre)", L"Edge radius Tool(micro metre)", "Radius according to P1 and P2");
	return 0;
}

