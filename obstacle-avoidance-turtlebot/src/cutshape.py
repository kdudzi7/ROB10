
# Python program to explain cv2.circle() method 
   
# importing cv2 
from PIL import Image
import cv2 
import numpy as np 
from matplotlib import pyplot as pp
from scipy import misc
import math



def sector_mask(shape,centre,radius,angle_range):
    

    x,y = np.ogrid[:shape[0],:shape[1]]
    cx,cy = centre
    tmin,tmax = np.deg2rad(angle_range)    
    if tmax < tmin:
            tmax += 2*np.pi    
    r2 = (x-cx)*(x-cx) + (y-cy)*(y-cy)
    theta = np.arctan2(x-cx,y-cy) - tmin   
    theta %= (2*np.pi)   
    circmask = r2 <= radius*radius   
    anglemask = theta <= (tmax-tmin)

    return circmask*anglemask

   


if __name__ == '__main__':	
	startingx = 205
	startingy = 218
	matrix = cv2.imread("/home/harumanager/map.pgm", cv2.IMREAD_COLOR)
	matrix1 = cv2.imread("/home/harumanager/map.pgm", cv2.IMREAD_COLOR)

	pp.imshow(matrix1)
	pp.show()
	mask = sector_mask(matrix.shape,(startingx,startingy,),10,(0,180))
	matrix[~mask] = 125
	number_of_white_pix = np.sum(matrix== 254)
	number_of_black_pix = np.sum(matrix == [0,0,0])
	
	pp.imshow(matrix, extent=[-10,10,-10,10])
	pp.show()
	print('Number of white pixels:', number_of_white_pix)
	print('Number of black pixels:', number_of_black_pix)
	if(number_of_black_pix > 0):
				print('There is an obstacle') 
	pp.imshow(matrix)
	#pp.imshow(mask)
	pp.show()
	print("lalala")
	value1 = input("Enter a value:\n")
	if(value1 == "w"):
		up = -2
		matrix = cv2.imread("/home/harumanager/map.pgm", cv2.IMREAD_COLOR)
		new_starting_x = startingx + up
		mask = sector_mask(matrix.shape,(startingx + up ,218),10,(0,180))

		matrix[~mask] = 125
		n = np.array(matrix)
		number_of_white_pix = np.sum(matrix== 254)
		number_of_black_pix = np.sum(matrix == [0,0,0])

		print('Number of white pixels:', number_of_white_pix)
		print('Number of black pixels:', number_of_black_pix)
		disntance_array = []
		angle_array = []
		if(number_of_black_pix > 0):
					print('There is an obstacle')
					xcoords, ycoords = np.where((n[:, :, 0:3] == [0,0,0]).all(2))
					for x in xcoords:
						for y in ycoords:
							print(x , y)
							matrix[xcoords,ycoords] = [254,0,0]
							matrix[startingx+up,startingy] = [60,60,60]
							number_of_green_pix = np.sum(matrix == [60,60,60])
							print('Number of green pixels:', number_of_green_pix)
							#254, 0, 0 = im.getpixel(134,179) 
							distance =  math.sqrt(((x - startingx)**2) + ((y - startingy)**2))
							disntance_array.append(distance)
							myradians = math.atan2(y - startingy, x - startingx+up)
							mydegrees = math.degrees(myradians)
							angle_array.append(mydegrees)
							
		smallest_distance = min(disntance_array)
		smallest_angle_index = disntance_array.index(min(disntance_array))
		angle_of_the_smallest_distance = angle_array[smallest_angle_index]
		print(smallest_distance ,angle_of_the_smallest_distance)
		pp.imshow(matrix)
		#pp.imshow(mask)
		pp.show()


