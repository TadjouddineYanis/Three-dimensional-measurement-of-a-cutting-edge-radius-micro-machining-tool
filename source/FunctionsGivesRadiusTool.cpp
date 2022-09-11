#include<iostream>
#include<vector>
#include<cmath>

#include "LineFitting.hpp"

using namespace std;

//Function to print a vector type double on the screen.
void print_vector(vector<double> vect){
	cout << "size : " << vect.size() << endl;
	for(double n : vect){
		cout << n << " ";
	}
	cout << endl;
}
//function to generate a random vector like linspace with python.
vector<double> linspace (double x_start, double x_end, double n_points){
	vector<double> vec;
	if (n_points == 0) return vec;
	if (n_points == 1){
		vec.push_back(x_start);
		return vec;
	}
	double pas = (x_end - x_start)/(n_points - 1);
	for(int i=0;i<n_points-1;i++){
		vec.push_back(x_start + pas*i);
	}
	vec.push_back(x_end);
	return vec;
}
//Function to calculate the finite difference between the points of radius ROI data and the radius of cylinder estimated.
vector<double> finite_difference(vector<double> x, vector<double> y){
	vector<double> res;
	for(int i=0; i<(x.size())-1;i++){
		res.push_back(round(abs((y[i+1] - y[i])/(x[i+1] - x[i]))));
	}
	return res;
}
//Function to verify if we have a linear model very visible or not
short check_linear_part(vector<double> diff, int nb_points, double epsilon){
	int linear(0); int compteur(0);
	for(int i=0;i<(diff.size())-1;i++){
		if((abs(diff[i+1] - diff[i])) < epsilon){
			compteur++;
			if (compteur == nb_points){
				linear = 1;
			}
		}else{
			compteur = 0;
		}
	}
	return linear;
}

//Function to extract the linear part or the data who we can modelise it following a linear model
//Indeed, we check the result with linearity, if we detect directly a linear part => we can extract the correspondant points (radius ROI and radius estimated).
//Else, we do another difference with the precedent difference found, and then, we can exctract the points.

//Function to extract the X data
vector<double> extract_points_linear_x(vector<double> diff, vector<double> x, short linear, short seuil_1, short seuil_2, short taille){
	vector<double> select_points, DIF;
	vector<double> vec; int point;
	double difference;
	for(int q=0;q<(diff.size())-1;q++){
		if(linear == 0){
			difference = abs(diff[q+1] - diff[q]);
			DIF.push_back(difference);
			if (difference < seuil_1){
				if(select_points.size() < taille){
					select_points.push_back(difference);
					vec.push_back(x[q]);
				}else{
					select_points = select_points;
					vec = vec;
				}
			} else if (difference >= seuil_1){
				if(select_points.size() > 20){
					select_points = select_points;
					vec = vec;
				}else{
					select_points.clear();
					vec.clear();
				}
			}
		}else{
			if (diff[q] < seuil_2){
				if(select_points.size() < taille){
					select_points.push_back(difference);
					vec.push_back(x[q]);
				}else{
					select_points = select_points;
					vec = vec;
				}
			}else if(diff[q] >= seuil_2){
				if(select_points.size() > 20){
					select_points = select_points;
					vec = vec;
				}else{
					select_points.clear();
					vec.clear();
				}
			}
			
		}
	}
	//cout << "deuxième_difference_x: " << endl;
	//print_vector(DIF);
	//cout << "selected_points: " << endl;
	//print_vector(select_points);
	return vec;
}
//Function to extract the Y data
vector<double> extract_points_linear_y(vector<double> diff, vector<double> y, short linear, short seuil_1, short seuil_2, short taille){
	vector<double> select_points, DIF;
	vector<double> vec; int point;
	double difference;
	for(int q=0;q<(diff.size())-1;q++){
		if(linear == 0){
			difference = abs(diff[q+1] - diff[q]);
			DIF.push_back(difference);
			if (difference < seuil_1){
				if(select_points.size() < taille){
					select_points.push_back(difference);
					vec.push_back(y[q]);
				}else{
					select_points = select_points;
					vec = vec;
				}
			} else if (difference >= seuil_1){
				if(select_points.size() > 20){
					select_points = select_points;
					vec = vec;
				}else{
					select_points.clear();
					vec.clear();
				}
			}
		}else{
			if (diff[q] < seuil_2){
				if(select_points.size() < taille){
					select_points.push_back(difference);
					vec.push_back(y[q]);
				}else{
					select_points = select_points;
					vec = vec;
				}
			}else if(diff[q] >= seuil_2){
				if(select_points.size() > 20){
					select_points = select_points;
					vec = vec;
				}else{
					select_points.clear();
					vec.clear();
				}
			}
			
		}
	}
	//cout << "deuxième_difference_y: " << endl;
	//print_vector(DIF);
	return vec;
}
//Function to compute the parameter a and b of the line fitting the data (x,y) : y=a*x+b
vector<double> robest_line_fitting(vector<double> x, vector<double> y, short seuil){
	vector<double> vec(2,0);
	auto lineFitting = make_shared<LineFittingProblem>();
	lineFitting->setData(x, y);
	//robest::MSAC * solver = new robest::MSAC();
	robest::LMedS * solver = new robest::LMedS();
	solver->solve(lineFitting, seuil);
	lineFitting->getResult(vec[0], vec[1]);
	return vec;
}
//Function to compute the equation : y=a*x+b
vector<double> equation_droite(vector<double> ab, vector<double> x){
	vector<double> vec;
	for(int i=0;i<x.size();i++){
		vec.push_back(ab[0]*x[i]+ab[1]);
	}
	return vec;
}
//Function to compute the edge radius equation : radius_estimated = ROI/(2*sin(M_PI/4 - alpha/4))
vector<double> radius_equation(vector<double> x, double alpha){
	vector<double> vec;
	for(int i=0;i<x.size();i++){
		vec.push_back(x[i]/(2*sin((M_PI/4)-(alpha/4))));
	}
	return vec;
}
//Function to compute the intersection point between the two linear equation (radius_estimated and the linear model estimated by robest)
vector<double> intersection(double a, double b, double alpha){
	vector<double> vec(2,0);
	vec[0] = (2*b*sin((M_PI/4)-(alpha/4)))/(1-(2*a*sin((M_PI/4)-(alpha/4))));
	vec[1] = (a*vec[0])+b;
	return vec;
}
