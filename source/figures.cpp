#include<iostream>
#include<vector>
#include<string>
#include "pbPlots.hpp"
#include "supportLib.hpp"

using namespace std;

//Function to plot simple data x and y
int plot(vector<double> x, vector<double> y, int color[3], const wchar_t* titre, const wchar_t* x_label, const wchar_t* y_label, string name){
	bool success;
	StringReference *errorMessage = new StringReference();
	RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();
	ScatterPlotSeries *data = GetDefaultScatterPlotSeriesSettings();
	data->xs = &x;
	data->ys = &y;
	data->linearInterpolation = false; // false or true
	data->pointType = toVector(L"dots"); // dots, crosses, filled triangles, pixels, triangles
	data->lineType = toVector(L"dotted"); // dotted, dashed, dotdash, longdash, solid
	data->lineThickness = 2; 
	data->color = CreateRGBColor(color[0], color[1], color[2]);
	ScatterPlotSettings *settings = GetDefaultScatterPlotSettings();
	settings->width = 600;
	settings->height = 400;
	settings->autoBoundaries = true;
	settings->autoPadding = true;
	settings->title = toVector(titre);
	settings->xLabel = toVector(x_label);
	settings->yLabel = toVector(y_label);
	settings->scatterPlotSeries->push_back(data);
	success = DrawScatterPlotFromSettings(imageReference, settings, errorMessage);
	if(success){
        vector<double> *pngdata = ConvertToPNG(imageReference->image);
        WriteToFile(pngdata, name);
        DeleteImage(imageReference->image);
	}else{
	    cerr << "Error: ";
        for(wchar_t c : *errorMessage->string){
            cerr << c;
        }
        cerr << endl;
	}
	return success ? 0 : 1;
}
//Function to plot 4 data y according to the same data x. the 5 vectors must have the same size
int plot4(vector<double> x, vector<double> y, vector<double> y2, vector<double> y3, vector<double> x4, vector<double> y4, int color[3], int color2[3], int color3[3], int color4[3], const wchar_t* titre, const wchar_t* x_label, const wchar_t* y_label, string name){
	bool success;
	StringReference *errorMessage = new StringReference();
	RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();
	ScatterPlotSeries *data = GetDefaultScatterPlotSeriesSettings();
	data->xs = &x;
	data->ys = &y;
	data->linearInterpolation = false; // false or true
	data->pointType = toVector(L"dots"); // dots, crosses, filled triangles, pixels, triangles
	data->lineType = toVector(L"dotted"); // dotted, dashed, dotdash, longdash, solid
	data->lineThickness = 7; 
	data->color = CreateRGBColor(color[0], color[1], color[2]);
	
	ScatterPlotSeries *data2 = GetDefaultScatterPlotSeriesSettings();
	data2->xs = &x;
	data2->ys = &y2;
	data2->linearInterpolation = false; 
	data2->pointType = toVector(L"filled triangles");
	data2->lineType = toVector(L"solid"); 
	data2->lineThickness = 0.5; 
	data2->color = CreateRGBColor(color2[0], color2[1], color2[2]);
	
	ScatterPlotSeries *data3 = GetDefaultScatterPlotSeriesSettings();
	data3->xs = &x;
	data3->ys = &y3;
	data3->linearInterpolation = false; 
	data3->pointType = toVector(L"filled triangles");
	data3->lineType = toVector(L"solid"); 
	data3->lineThickness = 0.5; 
	data3->color = CreateRGBColor(color3[0], color3[1], color3[2]);
	
	ScatterPlotSeries *data4 = GetDefaultScatterPlotSeriesSettings();
	data4->xs = &x4;
	data4->ys = &y4;
	data4->linearInterpolation = false; 
	data4->pointType = toVector(L"dots");
	data4->lineType = toVector(L"dotted"); 
	data4->lineThickness = 10; 
	data4->color = CreateRGBColor(color4[0], color4[1], color4[2]);
	
	ScatterPlotSettings *settings = GetDefaultScatterPlotSettings();
	settings->width = 600;
	settings->height = 400;
	settings->autoBoundaries = true;
	settings->autoPadding = true;
	settings->title = toVector(titre);
	settings->xLabel = toVector(x_label);
	settings->yLabel = toVector(y_label);
	settings->scatterPlotSeries->push_back(data);
	settings->scatterPlotSeries->push_back(data2);
	settings->scatterPlotSeries->push_back(data3);
	settings->scatterPlotSeries->push_back(data4);
	success = DrawScatterPlotFromSettings(imageReference, settings, errorMessage);
	if(success){
        vector<double> *pngdata = ConvertToPNG(imageReference->image);
        WriteToFile(pngdata, name);
        DeleteImage(imageReference->image);
	}else{
	    cerr << "Error: ";
        for(wchar_t c : *errorMessage->string){
            cerr << c;
        }
        cerr << endl;
	}
	return success ? 0 : 1;
}

int plot2(vector<double> x, vector<double> y, vector<double> y2, int color[3], int color2[3], const wchar_t* titre, const wchar_t* x_label, const wchar_t* y_label, string name){
	bool success;
	StringReference *errorMessage = new StringReference();
	RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();
	ScatterPlotSeries *data = GetDefaultScatterPlotSeriesSettings();
	data->xs = &x;
	data->ys = &y;
	data->linearInterpolation = false; // false or true
	data->pointType = toVector(L"dots"); // dots, crosses, filled triangles, pixels, triangles
	data->lineType = toVector(L"dotted"); // dotted, dashed, dotdash, longdash, solid
	data->lineThickness = 7; 
	data->color = CreateRGBColor(color[0], color[1], color[2]);
	
	ScatterPlotSeries *data2 = GetDefaultScatterPlotSeriesSettings();
	data2->xs = &x;
	data2->ys = &y2;
	data2->linearInterpolation = false; 
	data2->pointType = toVector(L"filled triangles");
	data2->lineType = toVector(L"solid"); 
	data2->lineThickness = 0.5; 
	data2->color = CreateRGBColor(color2[0], color2[1], color2[2]);
	
	ScatterPlotSettings *settings = GetDefaultScatterPlotSettings();
	settings->width = 600;
	settings->height = 400;
	settings->autoBoundaries = true;
	settings->autoPadding = true;
	settings->title = toVector(titre);
	settings->xLabel = toVector(x_label);
	settings->yLabel = toVector(y_label);
	settings->scatterPlotSeries->push_back(data);
	settings->scatterPlotSeries->push_back(data2);
	success = DrawScatterPlotFromSettings(imageReference, settings, errorMessage);
	if(success){
        vector<double> *pngdata = ConvertToPNG(imageReference->image);
        WriteToFile(pngdata, name);
        DeleteImage(imageReference->image);
	}else{
	    cerr << "Error: ";
        for(wchar_t c : *errorMessage->string){
            cerr << c;
        }
        cerr << endl;
	}
	return success ? 0 : 1;
}
