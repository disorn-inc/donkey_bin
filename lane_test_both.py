#!/usr/bin/env python3
from __future__ import print_function
import cv2
import numpy as np
import statistics
import sys
import rospy
from std_msgs.msg import String,Float64,Int64,Int8
import roslib
roslib.load_manifest('usb_cam')
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

lower_red = np.array([0, 50, 180])
upper_red = np.array([5, 255, 255])

lower_green = np.array([80,0,40])
upper_green = np.array([180,255,255])

#lower_green = np.array([103,0,100])
#upper_green = np.array([125,255,255])

lower_yellow = np.uint8([18, 35, 80])
upper_yellow = np.uint8([38, 255, 255])
# lower_yellow = np.array([25, 80,70])
# upper_yellow = np.array([40, 232, 255])

lower_white = np.uint8([0, 200, 0])
upper_white = np.uint8([255, 255, 255])

Servo_motor = 0
stateRight=0
slope = 1
intercept = 1
xpointbot = 0
xpointtop = 0 
xpointmid = 0
x_C = [0, 0, 0]
B1 = [0,0]
B2 = [0,0]
B3 = [0,0]
B4 = [0,0]
multiply_y = [0.9,0.8,0.7]
Size_crop = [0,0]
blobs = [0, 0, 0]
blob_c = [0, 0, 0]
keypoint = [0, 0, 0]
kernel = np.ones((3,3), np.uint8)
MODE_COLOR = 0
trig_mode=1#0 left, 1 right
bridge = CvBridge()



def intitial():
    rospy.Subscriber("/usb_cam/image_raw",Image,Getcam,queue_size = 1, buff_size=2**24)
    #rospy.Subscriber("/usb_cam/image_raw",Image,=main)

    # cap = cv2.VideoCapture(0)

    # while(True):
    #     # Capture frame-by-frame
    #     ret, frame = cap.read()

    #     # Our operations on the frame come here
    #     #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #     # Display the resulting frame
    #     Getcam(frame)
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break

    # # When everything done, release the capture
    # cap.release()
    # cv2.destroyAllWindows()

def callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # cv_image = data
    kernel = np.ones((5,5),np.float32)/25
    #crop_frame = cv_image[12:57 , 152:253]
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
    hsv_2 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("Image window", hsv)
    cv2.imshow("Image window2", hsv_2)
    cv2.waitKey(3)
    #image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def Getcam(data):
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # cv_image = data
    point(cv_image)
    #pts1 = np.float32([[B1], [B2], [B3], [B 4]])
    #pts1 = np.float32([[67,87],[157,92],[0,180],[163,180]])
    #cv2.imshow('raw',cv_image)
    """
    cv2.circle(cv_image,(0,int(cv_image.shape[0]*0.6)),1,(0,0,255),2) #btl
    cv2.circle(cv_image,(int(cv_image.shape[1]),int(cv_image.shape[0]*0.6)),1,(0,0,255),2) #btr
    cv2.circle(cv_image,(int(cv_image.shape[1]*0.15),int(cv_image.shape[0]*0.4)),1,(0,0,255),2) #tl
    cv2.circle(cv_image,(int(cv_image.shape[1]*0.87),int(cv_image.shape[0]*0.4)),1,(0,0,255),2) #tr
    """ #0.87
    pts1 = np.float32([[int(cv_image.shape[1]*0.15),int(cv_image.shape[0]*0.35)],[int(cv_image.shape[1]*0.815) \
    ,int(cv_image.shape[0]*0.4)],[0,int(cv_image.shape[0]*0.52)],[int(cv_image.shape[1]),int(cv_image.shape[0]*0.6)]])
    # tl, tr, btl, btr
    pts2 = np.float32([[0, 0], [Size_crop[0], 0], [0, Size_crop[1]], [Size_crop[0], Size_crop[1]]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    result = cv2.warpPerspective(cv_image, matrix, (Size_crop[0], Size_crop[1]))
    #cv2.imshow('perspect',result)
    perspectiveResult = result
    cv2.imshow('perspect',perspectiveResult)
    perspectiveResult = cv2.resize(result, (0,0), fx=4, fy=4)
    
    #cv2.imshow('kkk',result)

    global red_traffic
    global green_traffic
    global Servo_motor
    red_traffic = 0
    green_traffic=1600
    if (red_traffic >= 50) :
      run = 0
      result_traff1 = red_traffic
    if green_traffic> 1500 :
      run = 1
      result_traff1 = green_traffic
      pub_traff = rospy.Publisher('/light', Int8, queue_size=1)
      pub_traff.publish(run)

    cv2.circle(cv_image,(0,int(cv_image.shape[0]*0.6)),1,(0,0,255),2) #btl
    cv2.circle(cv_image,(int(cv_image.shape[1]),int(cv_image.shape[0]*0.6)),1,(0,0,255),2) #btr
    cv2.circle(cv_image,(int(cv_image.shape[1]*0.15),int(cv_image.shape[0]*0.35)),1,(0,0,255),2) #tl
    cv2.circle(cv_image,(int(cv_image.shape[1]*0.85),int(cv_image.shape[0]*0.4)),1,(0,0,255),2) #tr

    cv2.imshow('frame',cv_image[0:int(cv_image.shape[0]),0:int(cv_image.shape[1])])

    result1, crop1, servo1, sum1 = getROIRight(result)
    cv2.imshow("Right", result1)
    result, crop, servo, sum2 = getROILeft(result)
    cv2.imshow("Left", result)

    #Send Servo
    print("Right: ",servo1)
    print("Left: ",servo)
    if (sum1 > sum2 and sum1 >= 50000):
        print("Direction: Right")
        Servo_motor = servo1
        stateRight=1
    elif (sum2 >= 50000):
        print("Direction: Left")
        Servo_motor = servo
        stateRight=0
    pub = rospy.Publisher('/angle', Float64, queue_size=3)
    pub.publish(Servo_motor)
    rospy.loginfo(Servo_motor)


    # cv2.imshow('color',crop)
    cv2.waitKey(3)
    return result,crop


def point(cv_image):
    global B1
    global B2
    global B3
    global B4
    global Size_crop
    B1[0] = int(cv_image.shape[1]*0.3125)
    B2[0] = int(cv_image.shape[1]*0.6875)
    B3[0] = int(cv_image.shape[1]*0)
    B4[0] = int(cv_image.shape[1]*1)
    B1[1] = int(cv_image.shape[0]*0.45)
    B2[1] = int(cv_image.shape[0]*0.45)
    B3[1] = int(cv_image.shape[0]*0.694)
    B4[1] = int(cv_image.shape[0]*0.694) 
    Size_crop[0] = int(cv_image.shape[1]*0.4)
    Size_crop[1] = int(cv_image.shape[0])

def getROILeft(result):
    result = Crop_to_Cal_Left(result)
    result = cv2.resize(result, (0,0), fx=4, fy=4)
    if MODE_COLOR == 0 :
      hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    elif MODE_COLOR == 1:
      hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HLS)
      mask = cv2.inRange(hsv, lower_white, upper_white)
    crop = cv2.bitwise_and(result, result, mask=mask)
    crop = cv2.morphologyEx(crop, cv2.MORPH_OPEN, kernel)
    crop = cv2.dilate(crop,kernel,iterations = 1)
    crop = cv2.morphologyEx(crop, cv2.MORPH_CLOSE, kernel)
    x_mid = int(result.shape[1]*0.5)
    y_bot = int(result.shape[0]*(multiply_y[0]+0.025))
    y_top = int(result.shape[0]*(multiply_y[2]+0.025))
    y_mid = int((y_bot+y_top)/2)
    main_Y = [result.shape[0]*(multiply_y[0]+0.025),
                result.shape[0]*(multiply_y[1]+0.025),
                result.shape[0]*(multiply_y[2]+0.025)]
    cv2.line(result, (int(result.shape[1]*0.5), y_bot),(int(result.shape[1]*0.5), y_top), (0, 0, 255), 1,  8)
    blobs[0], keypoint[0] = bloblane(crop, multiply_y[0],10)
    blobs[1], keypoint[1] = bloblane(crop, multiply_y[1],10)
    blobs[2], keypoint[2] = bloblane(crop, multiply_y[2],10)
    blobby(keypoint,0)
    blobby(keypoint,1)
    blobby(keypoint,2)
    BLOB=np.concatenate((blobs[2],blobs[1],blobs[0]),axis=0)
    #Detect_Curve(result)
    x_C[0], x_C[1], x_C[2] = Find_slope(blob_c, main_Y, y_bot, y_top,)   
    cv2.line(result, (int(x_C[2]), y_mid),(int(result.shape[1]*0.5), y_mid), (180, 0, 255), 1,  8)
    cv2.line(result, (int(x_C[0]), y_bot),(int(x_C[1]), y_top), (20,255,150), 2,  8)
    servo_for_left = Cal_Servo(result,x_C[2])
    result =np.concatenate((result,BLOB),axis=0)
    sum_yellow = np.sum(BLOB)
    print("SUM Left: ", sum_yellow)
    crop = cv2.resize(BLOB, (0,0), fx=0.5, fy=0.5)
    result = cv2.resize(result, (0,0), fx=0.5, fy=0.5)
    #cv2.imshow("Left",result)
    return result, crop, servo_for_left, sum_yellow

def getROIRight(result):
    result = Crop_to_Cal_Right(result)
    result = cv2.resize(result, (0,0), fx=4, fy=4)
    if MODE_COLOR == 0 :
      hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    elif MODE_COLOR == 1:
      hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HLS)
      mask = cv2.inRange(hsv, lower_white, upper_white)
    crop = cv2.bitwise_and(result, result, mask=mask)
    crop = cv2.morphologyEx(crop, cv2.MORPH_OPEN, kernel)
    crop = cv2.dilate(crop,kernel,iterations = 1)
    crop = cv2.morphologyEx(crop, cv2.MORPH_CLOSE, kernel)
    x_mid_r = int(result.shape[1]*0.5)
    y_bot_r = int(result.shape[0]*(multiply_y[0]+0.025))
    y_top_r = int(result.shape[0]*(multiply_y[2]+0.025))
    y_mid_r = int((y_bot_r+y_top_r)/2)
    main_Y_r = [result.shape[0]*(multiply_y[0]+0.025),
                result.shape[0]*(multiply_y[1]+0.025),
                result.shape[0]*(multiply_y[2]+0.025)]
    cv2.line(result, (int(result.shape[1]*0.5), y_bot_r),(int(result.shape[1]*0.5), y_top_r), (0, 0, 255), 1,  8)
    blobs[0], keypoint[0] = bloblane(crop, multiply_y[0],10)
    blobs[1], keypoint[1] = bloblane(crop, multiply_y[1],10)
    blobs[2], keypoint[2] = bloblane(crop, multiply_y[2],10)
    blobby(keypoint,0)
    blobby(keypoint,1)
    blobby(keypoint,2)
    BLOB=np.concatenate((blobs[2],blobs[1],blobs[0]),axis=0)
    #Detect_Curve(result)
    x_C[0], x_C[1], x_C[2] = Find_slope(blob_c, main_Y_r, y_bot_r, y_top_r,)   
    cv2.line(result, (int(x_C[2]), y_mid_r),(int(result.shape[1]*0.5), y_mid_r), (180, 0, 255), 1,  8)
    cv2.line(result, (int(x_C[0]), y_bot_r),(int(x_C[1]), y_top_r), (20,255,150), 2,  8)
    servo_for_right = Cal_Servo(result,x_C[2])
    result =np.concatenate((result,BLOB),axis=0)
    sum_yellow = np.sum(BLOB)
    print("SUM Right: ", sum_yellow)
    crop = cv2.resize(BLOB, (0,0), fx=0.5, fy=0.5)
    result = cv2.resize(result, (0,0), fx=0.5, fy=0.5)
    #cv2.imshow("Right",result)
    return result, crop, servo_for_right, sum_yellow

def Crop_to_Cal_Right(result):
    trig_modqe = 1
    A1 = [0 , int(result.shape[0])+50]
    A2 = [int(result.shape[1]*0.15) , int(result.shape[0])]
    A3 = [int(result.shape[1]*0.68), int(result.shape[0]*0.9)] #org 0.9
    A4 = [int(result.shape[1]*1) , int(result.shape[0]*0.6)]
    if trig_mode ==0:
        crop_img = result[A3[1]:A1[1], A1[0]:A2[0]]
    else :
        crop_img = result[A3[1]:A1[1], A3[0]:A4[0]]
    return crop_img

def Crop_to_Cal_Left(result):
    trig_modqe = 1
    upperLeft = [0 , int(result.shape[0])+50]
    upperRight = [int(result.shape[1]*0.15) , int(result.shape[0])]
    lowerLeft = [int(result.shape[1]*0), int(result.shape[0]*0.9)] #org 0.9
    lowerRight = [int(result.shape[1]*0.3) , int(result.shape[0]*0.6)]
    if trig_mode ==0:
        crop_img = result[lowerLeft[1]:upperLeft[1], upperLeft[0]:upperRight[0]]
    else :
        crop_img = result[lowerLeft[1]:upperLeft[1], lowerLeft[0]:lowerRight[0]]
    return crop_img
# def Crop_to_Cal(result): #right
#     # global trig_modqe
#     A1 = [0 , int(result.shape[0])] 
#     A2 = [int(result.shape[1]*0.15) , int(result.shape[0])]
#     A3 = [int(result.shape[1]*0.9), int(result.shape[0]*0.85)]
#     A4 = [int(result.shape[1]) , int(result.shape[0]*0.6)]
#     crop_img = result[A3[1]:A1[1], A3[0]:A4[0]] #right y1,y2 x1,x2
#     return crop_img

def bloblane(image, y_point,minblobs):
    if minblobs == 0:
        minblobs  = image.shape[1]*0.08
    xp =  int(image.shape[1])
    yp1 = int(image.shape[0] * y_point)
    yp2 = int(image.shape[0] * (y_point+0.05))
    crop = image[yp1:yp2, 0:xp]
    crop = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    params = cv2.SimpleBlobDetector_Params()
    params.filterByInertia = False
    params.filterByConvexity = False
    params.filterByColor = False
    params.minArea = minblobs
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(crop)
    blank = np.zeros((1, 1))
    blobs = cv2.drawKeypoints(crop, keypoints, blank, (0, 0, 255),
    cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    return blobs, keypoints


def Find_slope(point_X, point_Y, y_1, y_2):
    global slope
    global intercept
    global xpointbot
    global xpointtop
    global xpointmid
    sum_1 = 0
    sum_2 = 0    
    y_mid = (y_1+y_2)/2
    x_bar = statistics.mean(point_X)
    y_bar = statistics.mean(point_Y)    
    for i in range(3):
        sum_1 = sum_1 + ((point_X[i]-x_bar) * (point_Y[i]-y_bar))
        sum_2 = sum_2 + ((point_X[i]-x_bar)**2)
    if sum_2 != 0:
        slope = sum_1/sum_2
    if slope != 0:
        intercept = y_bar - (slope*x_bar)
        xpointbot = (y_1-intercept)/slope
        xpointtop = (y_2-intercept)/slope
        xpointmid = (y_mid-intercept)/slope
    return xpointbot, xpointtop, xpointmid


def Detect_Curve(result):
    global x_C
    x_C[0], x_C[1], x_C[2] = Find_slope(blob_c, main_Y, y_bot, y_top,)   
    cv2.line(result, (int(x_C[2]), y_mid),(int(result.shape[1]*0.5), y_mid), (180, 0, 255), 1,  8)
    cv2.line(result, (int(x_C[0]), y_bot),(int(x_C[1]), y_top), (20,255,150), 2,  8)


def Cal_Servo(result,mid_point):
    servo_angle = (mid_point/result.shape[1])*100
    if servo_angle < 0:
        servo_angle=0
    if servo_angle > 100:
         servo_angle=100
    return servo_angle


def blobby(key_point,x):
    global blob_c
    if len(keypoint[x])==1:
        blob_c[x] = keypoint[x][0].pt[0]
    else :
        pass

def main(args):
  rospy.init_node('light_node', anonymous=True)
  intitial()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)
