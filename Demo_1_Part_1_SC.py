#SEED LAB Group 8
#Demo 1
# Spring 2021
# Some code from:
#https://www.philipzucker.com/aruco-in-opencv/

import numpy as np
import cv2
import cv2.aruco as aruco
import math

#Globals:
#1.33
CALIBRATION_CONSTANT = 1.33
#48.8
FOV = 54
TOffset = 6
#mtx = np.array([[2.6822003708394282e+03, 0., 1.5588865381021240e+03], [0., 2.6741978758743703e+03, 1.2303469240154550e+03], [0., 0., 1.]])
#distor = np.array([2.0426196677407879e-01, -3.3902097431574091e-01, -4.1813964792274307e-03, -1.0425257413809015e-02, 8.2004709580884308e-02])
#camera_matrix = np.array([[2.6822003708394282e+03, 0., 1.5588865381021240e+03], [0., 2.6741978758743703e+03, 1.2303469240154550e+03], [0., 0., 1.]])
#dist_co = np.array([2.0426196677407879e-01, -3.3902097431574091e-01, -4.1813964792274307e-03, -1.0425257413809015e-02, 8.2004709580884308e-02])
camera_matrix = np.array([[1., 0., 1.], [0., 1., 1.], [0., 0., 1.]])
dist_co = np.array([0., 0., 0., 0., 0.])

       
def findCenter(corners): # Get the center of the aruco image in x,y pixel coordinates by taking half the difference between the top left and bottom right pixel of the marker and adding that to the top left coordinate (bottom corner (lowest) with respect to (x,y) coordinates starting from top left of image at (0,0))
    return( (abs(((corners[0][0][2][0] - corners[0][0][0][0]))/2) + corners[0][0][0][0]),
            (abs(((corners[0][0][2][1] - corners[0][0][0][1]))/2) + corners[0][0][0][1]) )       
       

cv2.waitKey(0)

cap = cv2.VideoCapture(0)
i=0
while(i<1000):
    _, frame = cap.read()
    #h,w,c = frame.shape
    #cv2.imshow('frame',frame)
    i=i+1
    
    h, w, c = frame.shape
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    ref = w/2

    if corners: # if a marker is detected
            
            #Angle Detection
            frame = aruco.drawDetectedMarkers(frame, corners, ids)
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, None) 
            aruco.drawAxis(frame, camera_matrix, None, rvec, tvec, length=0.05)
            center = findCenter(corners)
            if(center[0] > ref):
                angle = abs((center[0] - ref)) * (((FOV+TOffset)/2)/(ref))
                angDisp = "Angle is -" + str(angle)
            if(center[0] < ref):
                angle = abs((ref - center[0])) * (((FOV+TOffset)/2)/(ref))
                angDisp = "Angle is +" + str(angle)
                
            #angle = abs(ref-center[0])*((THETA/2)/ref)
                
            cv2.putText(frame, angDisp, org = (0, 400), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color = (0, 255, 255))
              
            cv2.imshow('frame',frame)
            #cv2.waitKey(0)
            
            
    

    # Status Handling
    if(not corners):marker = "No marker found"
    else:marker = "Found a marker!"
    cv2.putText(frame, marker, org = (0, 300), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color = (0, 0, 0))
    frame = aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow('frame',frame)


    k = cv2.waitKey(1)
    if k == 27: break
    #print(h,w)
    
#print(frame)
cv2.imshow('frame',frame)

cv2.waitKey(0)
cv2.destroyAllWindows()   

cap.release()
cv2.destroyAllWindows()

