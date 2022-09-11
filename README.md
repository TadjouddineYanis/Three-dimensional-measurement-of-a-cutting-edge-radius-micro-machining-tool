# Three-dimensional-measurement-of-a-cutting-edge-radius-micro-machining-tool

The algorithms implemented make it possible to find the radius of the cutting edge of a micro machining tool. The programs have been worked on only one type of tool and are implemented in C++/PCL.


To execute the programs, we have to first delete the build directory and create another build directory every time we change computer or path file. Then in the new build directory, we can run cmake .. and make to compile and ./main to execute all the programs.

There are four important directories :

SOURCE : contains all .cpp files
HEADER : contains all .hpp files
NUAGES : contains all .pcd files
RADIUS_MEASURED : contains all the result in .txt file that we can plot with python 

All .cpp files are linked in the main.cpp file. First, we just need to choose the .pcd file in which we want to calculate the edge radius. Then, we give the ratio of the .pcd file allowing to convert pixels into micrometers. And the other important parameter is the threshold that can be taken between 0.001 and 0.5. There are effectivelly other parameters who we can change value, but it is not necessary. They are all explained in the main file. 

Once we have launched the program and the process has finished, it may take some time, there is a .py file in the source directory to draw the figures. It can be executed directly by : python3 ../source/figure.py

FIGURE.CPP : contains all functions to draw graphics

PBPLOTS.CPP : contains functions algorithms allow us to plot data

SUPPORTLIB.CPP : also contains functions for plotting
PBPLOTS.CPP and SUPPORTLIB.CPP are two files that I found online allow to plot data on C++ in 2D

VISUALISATION.CPP : contains functions to visualize 3D data with PCL

LINEFITTING.CPP : contains functions to estimate the linear model by LMedS algorithm

FUNCTIONSGIVESLISTSRADIUS.CPP : contains all functions allowing to estimate the list of the possible edge radius of thz tool.

FUNCTIONSGIVESRADIUSTOOL.CPP : contains all functions allowing to extract the good rafius among those estimated.

All functions are explained in their corresponding files.
