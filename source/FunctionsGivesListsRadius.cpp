#include<iostream>
#include<vector>
#include<string>
#include<cmath>
#include<random>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/angles.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

#include"visualisation.hpp"

using namespace std;

vector<double> roi_result, radius_result;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);

//Function to convert the point cloud to micro-meter with the ratio parameter
pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double ratio, double pourcentUsed){ 
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
	double paramPointsUnused = pourcentUsed/100;
	int i = 0;
	for (int p=0;p<cloud->points.size();p++){  
		if((rand()/(double)RAND_MAX)<paramPointsUnused){
			cloud2->push_back(cloud->points[p]);
			cloud2->points[i].x = ratio * cloud2->points[i].x;
			cloud2->points[i].y = ratio * cloud2->points[i].y; 
			cloud2->points[i].z = ratio * cloud2->points[i].z;
			i = i+1;
		}
	}
    	return cloud2;
}

//Function to add some noise on the cloud: white noise, normal noise, sinusoidal noise and abberant noise.
pcl::PointCloud<pcl::PointXYZ>::Ptr addNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string noiseType, double parameter){ 
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoisy (new pcl::PointCloud<pcl::PointXYZ>);
	double levelOfNoise = 0;
	for(int p=0;p<cloud->points.size();p++){  	
		cloudNoisy->push_back(cloud->points[p]); 
	}
	//Bruit Blanc
	if(noiseType == "bruitBlanc"){
		levelOfNoise = parameter;
		cout << "noiseType: " << noiseType << "; niveau de bruit: " << levelOfNoise << endl;
		for (int p=0;p<cloudNoisy->points.size();p++){  
			cloudNoisy->points[p].x = cloudNoisy->points[p].x + levelOfNoise*((rand()/(double)RAND_MAX)*2-1);
			cloudNoisy->points[p].y = cloudNoisy->points[p].y + levelOfNoise*((rand()/(double)RAND_MAX)*2-1);
			cloudNoisy->points[p].z = cloudNoisy->points[p].z + levelOfNoise*((rand()/(double)RAND_MAX)*2-1);
		}
	}
	// Bruit normal
	if(noiseType == "bruitNormal"){
		levelOfNoise = parameter/3;
		cout << "noiseType: " << noiseType << "; niveau de bruit: " << levelOfNoise << endl;
		std::default_random_engine generator;
		std::normal_distribution<double> distribution(0,levelOfNoise);
		for(int p=0;p<cloudNoisy->points.size();p++){  
			cloudNoisy->points[p].x = cloudNoisy->points[p].x + distribution(generator);
			cloudNoisy->points[p].y = cloudNoisy->points[p].y + distribution(generator);
			cloudNoisy->points[p].z = cloudNoisy->points[p].z + distribution(generator);
		}
	}
	//Bruit sinusoidal
	if( noiseType == "bruitSinusoidal"){
		levelOfNoise = parameter;
		cout << "noiseType: " << noiseType << "; niveau de bruit: " << levelOfNoise << endl;
		//double periodSinuX = 20;
		double periodSinuY = 20;
		double periodSinuZ = 20;
		periodSinuY = 20;//+((rand()/(double)RAND_MAX)*2-1)*5;
		periodSinuZ = 20;//+((rand()/(double)RAND_MAX)*2-1)*5;
		double theta = (rand()/(double)RAND_MAX)*2*M_PI;
		for (int p=0;p<cloudNoisy->points.size();p++){  
			cloudNoisy->points[p].y =  cloudNoisy->points[p].y  + levelOfNoise*sin(theta + 2 * M_PI / 10 + 2 * M_PI * cloudNoisy->points[p].x / periodSinuY);
			cloudNoisy->points[p].z =  cloudNoisy->points[p].z  + levelOfNoise*sin(theta + 2 * M_PI / 10 + M_PI / 2 + 2 * M_PI * cloudNoisy->points[p].x / periodSinuZ);
		}
	}
	// Bruit aberrant
	if(noiseType == "bruitAberrant"){
		levelOfNoise = parameter;//0.1+(n-1)//10;
		cout << "noiseType: " << noiseType << "; niveau de bruit: " << levelOfNoise << endl;
		for(int p=0;p<cloudNoisy->points.size();p++){ 
			double probaAberrant = rand()/(double)RAND_MAX;
			if (probaAberrant < levelOfNoise){	
				cloudNoisy->points[p].x = cloudNoisy->points[p].x + 100;//*((rand()/(double)RAND_MAX)*2-1);
				cloudNoisy->points[p].y = cloudNoisy->points[p].y + 100;//*((rand()/(double)RAND_MAX)*2-1);
				cloudNoisy->points[p].z = cloudNoisy->points[p].z + 100;//*((rand()/(double)RAND_MAX)*2-1);
			}
		}
	}
	return cloudNoisy;
}

//Function to estimate two perpendicular planes, between the two points P1 and P2.
pcl::ModelCoefficients::Ptr EstimatePerpendicularPlaneFrom2Points(pcl::PointXYZ P1, pcl::PointXYZ P2){
    	//calculate of the normal of the intersection line of the two planes
    	double n1=P2.x-P1.x;
	double n2=P2.y-P1.y;
	double n3=P2.z-P1.z;
    	//calculate the fourth parameter of the new plan
    	double n4=(-1)*((n1*P1.x) + (n2*P1.y) +(n3*P1.z));
	if(n4<0){
		n1 = -n1; n2 = -n2; n3 = -n3; n4 = -n4;
	}
	//cout << n1 << "   " << n2 << "   " << n3 << "   " << n4 << endl;
    	//calculate of the normal of the intersection line of the first and third plane
    	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    	coefficients->values.resize(4);
    	coefficients->values[0]=n1;
    	coefficients->values[1]=n2;
    	coefficients->values[2]=n3;
    	coefficients->values[3]=n4;
    	return coefficients;
}
//Function to compute the position of the plane.
double positionPointPlane(pcl::PointXYZ &P, pcl::ModelCoefficients::Ptr coefficients){
	if((coefficients->values[0]*P.x+coefficients->values[1]*P.y+coefficients->values[2]*P.z+coefficients->values[3])<0)
        	return -1;
    	else if((coefficients->values[0]*P.x+coefficients->values[1]*P.y+coefficients->values[2]*P.z+coefficients->values[3])>0)
        	return 1;
    	else 
    		return 0;
}
//Function to estimate the global ROI selected between the two points P1 and P2.
pcl::PointCloud<pcl::PointXYZ>::Ptr Global_ROI(pcl::PointXYZ P1, pcl::PointXYZ P2, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	pcl::ModelCoefficients::Ptr plan1(new pcl::ModelCoefficients);
  	pcl::ModelCoefficients::Ptr plan2(new pcl::ModelCoefficients);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr region_1 (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr region_2 (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr region_3 (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr ROI (new pcl::PointCloud<pcl::PointXYZ>);
  	plan1 = EstimatePerpendicularPlaneFrom2Points(P1, P2);
  	plan2 = EstimatePerpendicularPlaneFrom2Points(P2, P1);
  	double pos1, pos2;
  	for(size_t i=0;i<cloud->points.size();++i){
  		pos1 = positionPointPlane(cloud->points[i], plan1);
  		pos2 = positionPointPlane(cloud->points[i], plan2);
  		//cout << "position:  " << pos1 << "    " << pos2 << endl;
    		if((pos1 == -1) && (pos2 == 1)){    
			region_1->push_back(cloud->points[i]);
        	}else if ((pos1 == 1) && (pos2 == -1)){  
        		region_2->push_back(cloud->points[i]);
        	}else if ((pos1 == 1) && (pos2 == 1)){  
        		region_3->push_back(cloud->points[i]);
        	}
        }
        if(region_1->points.size() > 20) ROI = region_1;
        if(region_2->points.size() > 20) ROI = region_2;
        if((region_1->points.size() > 20) && (region_2->points.size() > 20) && (region_3->points.size() > 20)) ROI = region_3;
        return ROI;
}
//Function to draw the line between P1 and P2.
pcl::PointCloud<pcl::PointXYZ>::Ptr Line_P1_P2(pcl::PointXYZ P1, pcl::PointXYZ P2){
	double a, b, c, d;
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_droite (new pcl::PointCloud<pcl::PointXYZ>);
        a = (P2.y - P1.y)/(P2.x - P1.x);
        b = P1.y - (a*P1.x);
        c = (P2.z - P1.z)/(P2.x - P1.x);
        d = P1.z - (c*P1.x);
        for (float i=P1.x;i<=P2.x;i=i+0.1){
		pcl::PointXYZ point;
		point.x = i;
		point.y = (a*i)+b ;
		point.z = (c*i)+d;
		points_droite->points.push_back(point);
	}
	return points_droite;
}
//Function to draw the cylinder according to the radius ROI.
pcl::PointCloud<pcl::PointXYZ>::Ptr generateCircleData(const double cx, const double cy, const double cz, const double r){
	pcl::PointCloud<pcl::PointXYZ>::Ptr circle (new pcl::PointCloud<pcl::PointXYZ>);
    	for(float angle(0.0); angle <= 360.0; angle += 5.0){
        	pcl::PointXYZ basic_point;		
		basic_point.x = cx;
		basic_point.y = r*cos(pcl::deg2rad(angle)) + cy;
		basic_point.z = r*sin(pcl::deg2rad(angle)) + cz;
		circle->points.push_back(basic_point);
    	}
    	return circle;
}
//Function to compute de distance between a point to a line.
double DistancePoinToLine(pcl::PointXYZ  A, pcl::PointXYZ B, pcl::PointXYZ F){
	double xU = A.x-B.x;
	double yU = A.y-B.y;
	double zU = A.z-B.z;
	double xMod = (F.y-A.y)*(A.z-B.z)-(F.z-A.z)*(A.y-B.y);
	double yMod = (F.z-A.z)*(A.x-B.x)-(F.x-A.x)*(A.z-B.z);
	double zMod = (F.x-A.x)*(A.y-B.y)-(F.y-A.y)*(A.x-B.x);
    	return  std::sqrt(xMod*xMod+yMod*yMod+zMod*zMod)/std::sqrt(xU*xU + yU*yU + zU*zU);
}
//Function to extract the points within the cylinder with radius ROI.
pcl::PointCloud<pcl::PointXYZ>::Ptr area_interest_line(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double distance, pcl::PointXYZ P1, pcl::PointXYZ P2){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);
    	for (size_t i = 0; i < cloud->points.size() ; ++i){
    		if((DistancePoinToLine(P1,P2,cloud->points[i]) <= distance)){
    			cloud3->push_back(cloud->points[i]);
        	}
        }
    	return cloud3;
}
//Function to estimate with the points selected in the cylinder with radius ROI, the radius looking for, by segmentation.
pcl::ModelCoefficients::Ptr radius_estimated(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, float seuil, float iteration){
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  	pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  	// Estimate a normal vector	
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud);
	ne.setKSearch (50);
	ne.compute(*cloud_normals);	
	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients (true);
  	seg.setModelType (pcl::SACMODEL_CYLINDER);
  	seg.setMethodType (pcl::SAC_LMEDS);
  	seg.setMaxIterations (iteration);
  	seg.setDistanceThreshold (seuil);
  	seg.setRadiusLimits (1, 100);
  	seg.setInputCloud (cloud->makeShared());
  	seg.setInputNormals (cloud_normals);
  	seg.segment (*inliers_cylinder, *coefficients_cylinder);
  	// Write the cylinder inliers to disk
  	extract.setInputCloud (cloud);
  	extract.setIndices (inliers_cylinder);
  	extract.setNegative (false);
  	extract.filter (*cloud_cylinder);
  	return coefficients_cylinder;
}
//Function to estimate radius ROI and radius of cylinder estimated according to the global ROI.
//We define a point M(x, y, z) and then we compute de radius ROI, equal the distance between M and P2.
//Then, we increase the position of the point M, and we can take the points within the cylinder with radius ROI.
//Then, we compute the radius of cylinder estimated with the points selected.
void estimate_all_radius(pcl::PointCloud<pcl::PointXYZ>::Ptr ROI, pcl::PointCloud<pcl::PointXYZ>::ConstPtr points_drt, pcl::PointXYZ P1, pcl::PointXYZ P2, float numberStep, float pas, int disp, float seuil, float iteration){
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_cylindre (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cercle;
	pcl::PointCloud<pcl::PointXYZ>::Ptr droite;
	pcl::ModelCoefficients::Ptr coeff_cyl;
	roi_result.clear(); // we make sure  that the vector is empty before storing (because it is a global variable)
	radius_result.clear();
	int iter(0); double r_roi, radius_estim; double j(-1);
	
	while(j<numberStep){
		j=j+1;
		r_roi=pas*j;
		droite = area_interest_line(ROI, r_roi, P1, P2);
		cout << "iteration: " << j << "; size_Points_Cylinder: " << droite->points.size() << endl;
		if(droite->points.size()>=12){
			coeff_cyl = radius_estimated(droite, seuil, iteration);
			if((coeff_cyl->values[6]<0) || (coeff_cyl->values[6]>1000000)){
				cout << "Erreur de segmentation" << endl;
			}else{
				radius_estim = coeff_cyl->values[6];
				roi_result.push_back(r_roi);
				radius_result.push_back(radius_estim);
				cout << "r_roi: " << r_roi << endl;
				cout << "radius estimated: " << radius_estim << endl;
				if(disp==1){
					for(const auto& pt: *points_drt){ //to visualize the cylinder with radius ROI
						cercle = generateCircleData(pt.x, pt.y, pt.z, r_roi);
						for(const auto& u: *cercle){
							pcl::PointXYZ k;
							k.x = u.x;
							k.y = u.y;
							k.z = u.z;
  							points_cylindre->points.push_back(k);
  						}	
					}
					visualisation_ROI_cylindre(ROI, points_cylindre, cloud_cylinder);
				}
			}
		}
	
	}
	
	/*for(float k=0.2;k<numberStep;k=k+pas){
		pcl::PointXYZ M;
		M.x = P2.x;
		M.y = P2.y - k;
		M.z = P2.z + k;
		r_roi = sqrt(pow(M.x - P2.x, 2) + pow(M.y - P2.y, 2) + pow(M.z - P2.z, 2));
		for(const auto& pt: *points_drt){ //to visualize the cylinder with radius ROI
			cercle = generateCircleData(pt.x, pt.y, pt.z, r_roi);
			for(const auto& u: *cercle){
				pcl::PointXYZ k;
				k.x = u.x;
				k.y = u.y;
				k.z = u.z;
  				points_cylindre->points.push_back(k);
  			}	
		}
		droite = area_interest_line(ROI, r_roi, P1, P2);
		coeff_cyl = radius_estimated(droite, seuil, iteration);
		radius_estim = abs(coeff_cyl->values[6]);
		roi_result.push_back(r_roi);
		radius_result.push_back(radius_estim);
		iter++;
		cout << "iteration: " << iter << "; valeur_boucle: " << k << "; size_Points_Cylinder: " << droite->points.size() << endl;
		cout << "r_roi: " << r_roi << endl;
		cout << "radius estimated: " << radius_estim << endl;
		if(disp==1) visualisation_ROI_cylindre(ROI, points_cylindre, cloud_cylinder);
	}*/
}
//Function to stock the roi values.
vector<double> ROI_values(void){
	return roi_result;
}
//Function to stock the estimated radius values.
vector<double> estimated_radius_values(void){
	return radius_result;
}
//Function to write de data ROI and the estimated radius within a .txt file.
void write_file(vector<double> x, vector<double> y, string path, string name_roi, string name_radius_estim){
	ofstream output_fileX(path+name_roi+".txt");
	ostream_iterator<double> output_iteratorX(output_fileX,",\n");
	copy(x.begin(), x.end(), output_iteratorX);
	
	ofstream output_fileY(path+name_radius_estim+".txt");
	std::ostream_iterator<double> output_iteratorY(output_fileY,",\n");
    	std::copy(y.begin(), y.end(), output_iteratorY);
}
//Function to compute the distance between P1 and P2.
double distance_P1_P2(pcl::PointXYZ P1, pcl::PointXYZ P2){
	return sqrt(pow(P1.x - P2.x, 2) + pow(P1.y - P2.y, 2) + pow(P1.z - P2.z, 2));
}


