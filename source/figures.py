import matplotlib.pyplot as plt

path = "../radius_measured/"
roi = open(path+"roi.txt", "r")
radius_estim = open(path+"radius.txt", "r")
roi_filtered = open(path+"roi_filtered.txt", "r")
radius_estim_filtered = open(path+"radius_filtered.txt", "r")
roi_extract = open(path+"roi_extract.txt", "r")
radius_estim_extract = open(path+"radius_extract.txt", "r")
robest_line = open(path+"robest_line.txt", "r")
#radius_equation = open(path+"radius_equation.txt", "r")
x_i = open(path+"inter_x.txt", "r")
y_i = open(path+"inter_y.txt", "r")
dist = open(path+"distance.txt", "r")
edge = open(path+"edge_radius.txt", "r")


r_roi = []
r_radius = []
for j in roi:
	data = [str(elt) for elt in j.split(",")]
	r_roi.append(float(data[0]))
	
for j in radius_estim:
	data = [str(elt) for elt in j.split(",")]
	r_radius.append(float(data[0]))

plt.subplot(221)
plt.scatter(r_roi, r_radius, color='blue')
#plt.xlabel("roi (µm)")
plt.ylabel("edge radius (µm)")
plt.title("Graph of data ROI and estimated Radius")

r_roi_filtered = []
r_radius_filtered = []
for j in roi_filtered:
	data = [str(elt) for elt in j.split(",")]
	r_roi_filtered.append(float(data[0]))
	
for j in radius_estim_filtered:
	data = [str(elt) for elt in j.split(",")]
	r_radius_filtered.append(float(data[0]))

plt.subplot(222)
plt.scatter(r_roi_filtered, r_radius_filtered, color='blue')
#plt.xlabel("roi (µm)")
plt.ylabel("edge radius (µm)")
plt.title("Graph of data ROI and estimated Radius filtered")

roi_ext = []
radius_ext = []
for j in roi_extract:
	data = [str(elt) for elt in j.split(",")]
	roi_ext.append(float(data[0]))
	
for j in radius_estim_extract:
	data = [str(elt) for elt in j.split(",")]
	radius_ext.append(float(data[0]))

plt.subplot(223)
plt.scatter(roi_ext, radius_ext, color='blue')
plt.xlabel("roi (µm)")
plt.ylabel("edge radius (µm)")
plt.title("Graph of data selectionned")


robest = []
#r_equation = []
xxi = []
yyi = []
for j in robest_line:
	data = [str(elt) for elt in j.split(",")]
	robest.append(float(data[0]))
	
#for j in radius_equation:
#	data = [str(elt) for elt in j.split(",")]
#	r_equation.append(float(data[0]))
for j in x_i:
	data = [str(elt) for elt in j.split(",")]
	xxi.append(float(data[0]))
	
for j in y_i:
	data = [str(elt) for elt in j.split(",")]
	yyi.append(float(data[0]))

plt.subplot(224)
plt.scatter(roi_ext, radius_ext, color='blue')
plt.plot(roi_ext, robest, color='red')
#plt.plot(roi_ext, r_equation, color='green')
plt.scatter(xxi, yyi, color='green')
plt.xlabel("roi (µm)")
plt.ylabel("edge radius (µm)")
plt.title("Edge radius measurement of a micro usinage tool")
plt.legend(['robest_line', 'extract point', 'intersection'])
plt.show()

distance = []
edge_radius = []
for j in dist:
	data = [str(elt) for elt in j.split(",")]
	distance.append(float(data[0]))
	
for j in edge:
	data = [str(elt) for elt in j.split(",")]
	edge_radius.append(float(data[0]))

plt.scatter(distance, edge_radius, color='blue')
plt.xlabel("distance P1 and P2 (µm)")
plt.ylabel("edge radius (µm)")
plt.title("Edge radius evolution according to the points P1 and P2")
plt.show()

plt.scatter(roi_ext, radius_ext, color='blue')
plt.plot(roi_ext, robest, color='red')
#plt.plot(roi_ext, r_equation, color='green')
plt.scatter(xxi, yyi, color='green')
plt.xlabel("roi (µm)")
plt.ylabel("edge radius (µm)")
plt.title("Edge radius measurement of a micro usinage tool")
plt.legend(['robest_line', 'extract point', 'intersection'])
plt.show()
