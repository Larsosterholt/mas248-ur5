#!/usr/bin/env python

# https://github.com/bigdayangyu/UR5-project/blob/master/ur5_project.m

import numpy as np
import cv2
from matplotlib import pyplot as plt
img = cv2.imread("src/mas248project/scripts/sir_isaac.png")

#cv2.imshow('original_image',img)
#waits for user to press any key 
#(this is necessary to avoid Python kernel form crashing)
#cv2.waitKey(0) 
#cv2.destroyAllWindows() 

blur = cv2.GaussianBlur(img,(5,5),cv2.BORDER_DEFAULT)
# STEP -1 Convert img from BGR Space to Grayscale space



imgray = cv2.cvtColor(blur,cv2.COLOR_BGR2GRAY)
#cv2.imshow('grayscale_image',imgray)
#cv2.waitKey(0)   
#cv2.destroyAllWindows() 





thresh_image = cv2.adaptiveThreshold(imgray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,5)
#cv2.imshow('thresh_image',thresh_image)
#cv2.waitKey(0)   
#cv2.destroyAllWindows()


# Find Canny edges
edged = cv2.Canny(thresh_image, 30, 200)

#cv2.imshow('edged',edged)
#cv2.waitKey(0)   
#cv2.destroyAllWindows() 



# Write function to find contours
# - Retrieval - TREE
# - Approximation - Simple
contours, hierarchy = cv2.findContours(edged,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

# drawing contours over blank image
contours_image = np.zeros(img.shape, dtype=np.uint8);
cv2.drawContours(contours_image, contours, -1, (0,255,0), 3)

#cv2.imshow('contours_image',contours_image)
#cv2.waitKey(0)   
#cv2.destroyAllWindows() 

print("Number of Contours found = " + str(len(contours)))


print(np.array(contours).size)


x_coord = []
y_coord = []
z_coord = []
for cont in contours:
	pixel_pose = cont.reshape(-1,2)
	#print ('pixel_pose',pixel_pose)
	x_coord.extend(pixel_pose[:,1])
	y_coord.extend(pixel_pose[:,0])

	z_coord_zero = [0.0 for i in pixel_pose]
	z_coord.extend(z_coord_zero)


	
	x_coord.extend([pixel_pose[-1,1]])
	y_coord.extend([pixel_pose[-1,0]])
	z_coord.extend([0.2])


	#print('len(x_coord)',len(x_coord))

	#print ('x_coord',x_coord)



#scale the image to reasonable size
print("Image size = ",img.shape)
x_coord = (-min(x_coord) + x_coord)/(max(x_coord)-min(x_coord))*0.210 + 0.500;  #Add offset
y_coord = (-min(y_coord) + y_coord)/(max(y_coord)-min(y_coord))*0.290;

#x_coord[:] = [x/ img.shape[0]*0.210 + 0.20 for x in x_coord]
#y_coord[:] = [y/ img.shape[1]*0.290 + 0.20 for y in y_coord]



#print('x_coord',x_coord)
#print('y_coord',y_coord)
#print('z_coord',z_coord)

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_coord, y_coord,z_coord)
plt.show()
