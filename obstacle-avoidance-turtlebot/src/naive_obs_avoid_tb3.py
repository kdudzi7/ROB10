#!/usr/bin/env python
#import rospy # Python library for ROS
#from nav_msgs.msg import Odometry

#def callback(msg):
    #print msg.pose.pose

#rospy.init_node('check_odometry')
#odom_sub = rospy.Subscriber('/odom', Odometry, callback)
#rospy.spin()
#!/usr/bin/env python
from PIL import Image
import cv2 
import numpy as np 

def move():
	value = input("Enter a value:\n")
	print(value)

def main():
	im = Image.open("/home/harumanager/map.pgm") # Can be many different formats.
	pix = im.load()	
	im.show() 
	print (im.size)
	width, height = im.size   # Get the width and hight of the image for iterating over
	#print (pix[45,70])  # Get the RGBA Value of the a pixel of an image
	rgb_pixel_value = im.getpixel((165,152))
	print(rgb_pixel_value)
	print ( (255 - pix[165, 152]) / 255.0)
	print('I am hereee')
	left = 5
	top = height / 4
	right = 164
	bottom = 3 * height / 4
	img = cv2.imread("/home/harumanager/map.pgm")
	
	img = cv2.rectangle(img, (205,215), (218,230), (1,255,1), 1)
	#pil_draw.pieslice((0, 0, pil_size-1, pil_size-1), 330, 0, fill=GREY)


	cv2.imshow('Draw01',img)
	imcropped = img[215:230, 205:218]
	cv2.imshow("cropped", imcropped)



	#imcropped = img.crop((160, 150, 170, 185))
	#imcropped.show()	
	number_of_white_pix = np.sum(imcropped == 254)
	number_of_black_pix = np.sum(imcropped == [0,0,0])
	print('Number of white pixels:', number_of_white_pix)
	print('Number of black pixels:', number_of_black_pix)
	height1, width1 , channels1 = imcropped.shape
	print(imcropped.shape)
	print('Height',height1)
	input = move()
	for i in range(height1):
		for j in range(width1):
			b,g,r = (imcropped[i, j])
			#rgb_pixel_value = imcropped.getpixel((i,j))
			#print('width1',i)
			#print('height1',j)
			#print(rgb_pixel_value)
			#print (r)
			#print (g)
			#print (b)
			#input = move()
			#print(input)
			value1 = "w"			
			#v3alue1 = input("Enter a value:\n")
			print("declared value is ", value1)
			if value1 in["w" , "a"] :
				move_forward = -2
				imcropped = img[215 + move_forward :230 + move_forward, 205:218]
				cv2.imshow("cropped2", imcropped)
				number_of_white_pix = np.sum(imcropped == 254)
				number_of_black_pix = np.sum(imcropped == [0,0,0])
				print('Number of white pixels in moved :', number_of_white_pix)
				print('Number of black pixels in moved:', number_of_black_pix)



			if(number_of_black_pix > 0):
				print('There is an obstacle' , i , j)         #looping at python speed...
		
	cv2.waitKey(0)

if __name__ == '__main__':	
    main()