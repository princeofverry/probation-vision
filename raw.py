from scipy.spatial import distance as dist
from collections import deque
import numpy as np
import argparse
from imutils import perspective
import imutils
import cv2
import math
import urllib #for reading image from URL
import serial

ser1 = serial.Serial('/dev/cu.usbmodem1D11401', 9600)
# def nothing(x):
#     pass
# cv2.namedWindow("Trackbars")

# cv2.createTrackbar("L – H Red", "Trackbars", 0, 179, nothing)
# cv2.createTrackbar("L – S Red", "Trackbars", 0, 255, nothing)
# cv2.createTrackbar("L – V Red", "Trackbars", 0, 255, nothing)
# cv2.createTrackbar("U – H Red", "Trackbars", 179, 179, nothing)
# cv2.createTrackbar("U – S Red", "Trackbars", 255, 255, nothing)
# cv2.createTrackbar("U – V Red", "Trackbars", 255, 255, nothing)

# cv2.createTrackbar("L – H Green", "Trackbars", 0, 179, nothing)
# cv2.createTrackbar("L – S Green", "Trackbars", 0, 255, nothing)
# cv2.createTrackbar("L – V Green", "Trackbars", 0, 255, nothing)
# cv2.createTrackbar("U – H Green", "Trackbars", 179, 179, nothing)
# cv2.createTrackbar("U – S Green", "Trackbars", 255, 255, nothing)
# cv2.createTrackbar("U – V Green", "Trackbars", 255, 255, nothing)
def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)
# url = 'http://192.168.0.4:8080/video'
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
    help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
    help="max buffer size")
# ap.add_argument("-w", "--width", type=float, default=2.5,
#     help="width of the left-most object in the image (in inches)")
args = vars(ap.parse_args())

# object
width = 7.87 # (inch)
 
# define the lower and upper boundaries of the colors in the HSV color space
# hijau 61 167 90

# hijau 161 155 84 low
# hijau 102, 255, 255 high
lower = {
    'red': (0, 127, 90),
    'green': (62, 82, 0),
    'blue': (90, 150, 140),
    'yellow': (23, 59, 119),
    'orange': (0, 130, 190),
}
lower['blue'] = (93, 10, 0)

upper = {
    'red': (5, 255, 255),
    'green': (99, 255, 245),
    'blue': (102, 255, 255),
    'yellow': (54, 255, 255),
    'orange': (5, 255, 255)
}

# define standard colors for circle around the object
colors = {
'red':(179,255,255), 
'green':(109,255,255), 
'blue':(255,0,0), 
'yellow':(0, 255, 217), 
'orange':(0,140,255)
}

#pts = deque(maxlen=args["buffer"])
 
# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
    # camera = cv2.VideoCapture(0)
    camera = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! nvvidconv ! video/x-raw(memory:NVMM) ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1 ", cv2.CAP_GSTREAMER)
# otherwise, grab a reference to the video file
else :
    camera = cv2.VideoCapture(args["video"])


#       key value
# camera.set(3 , 640  ) # width        
# camera.set(4 , 480  ) # height       
# camera.set(10, 0  ) # brightness     min: 0   , max: 255 , increment:1  
# camera.set(11, 50   ) # contrast       min: 0   , max: 255 , increment:1     
# camera.set(12, 70   ) # saturation     min: 0   , max: 255 , increment:1
# camera.set(13, 13   ) # hue         
# camera.set(14, 5   ) # gain           min: 0   , max: 127 , increment:1
# camera.set(15, -7   ) # exposure       min: -7  , max: -1  , increment:1
# # camera.set(17, 5000 ) # white_balance  min: 4000, max: 7000, increment:1
# camera.set(28, 0) # focus          min: 0   , max: 255 , increment:5
nilaitengah = str(320)
tolerance_value = 30
pointsList = []
# keep looping
while True:

    # grab the current frame
    camera.set(28, 255)
    # camera.set(15, -1)
    # camera.set(10, 100)
    (grabbed, frame) = camera.read()
    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    # if args.get("video") and not grabbed:
    #     break

    #IP webcam image stream 
    #URL = 'http://10.254.254.102:8080/shot.jpg'
    #urllib.urlretrieve(URL, 'shot1.jpg')
    #frame = cv2.imread('shot1.jpg')

 
    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=640, height=960)

    # l_h_red = cv2.getTrackbarPos("L – H Red", "Trackbars")
    # l_s_red = cv2.getTrackbarPos("L – S Red", "Trackbars")
    # l_v_red = cv2.getTrackbarPos("L – V Red", "Trackbars")
    # u_h_red = cv2.getTrackbarPos("U – H Red", "Trackbars")
    # u_s_red = cv2.getTrackbarPos("U – S Red", "Trackbars")
    # u_v_red = cv2.getTrackbarPos("U – V Red", "Trackbars")

    # l_h_green = cv2.getTrackbarPos("L – H Green", "Trackbars")
    # l_s_green = cv2.getTrackbarPos("L – S Green", "Trackbars")
    # l_v_green = cv2.getTrackbarPos("L – V Green", "Trackbars")
    # u_h_green = cv2.getTrackbarPos("U – H Green", "Trackbars")
    # u_s_green = cv2.getTrackbarPos("U – S Green", "Trackbars")
    # u_v_green = cv2.getTrackbarPos("U – V Green", "Trackbars")

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # lower_red = np.array([l_h_red, l_s_red, l_v_red])
    # upper_red = np.array([u_h_red, u_s_red, u_v_red])
    # mask_red = cv2.inRange(hsv, lower_red, upper_red)

    # result_red = cv2.bitwise_and(frame, frame, mask=mask_red)

    # lower_green = np.array([l_h_green, l_s_green, l_v_green])
    # upper_green = np.array([u_h_green, u_s_green, u_v_green])
    # mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # result_green = cv2.bitwise_and(frame, frame, mask=mask_green)

##HERE
    kernel1 = np.ones((9,9), np.uint8)
    mask_red = cv2.inRange(hsv, lower['red'], upper['red']) 
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel1)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel1)
    

    cnts_red = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
    # center_red = None

    kernel2 = np.ones((9,9), np.uint8)
    mask_green = cv2.inRange(hsv, lower['green'], upper['green'])
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel2)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel2)

    cnts_green = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]

    col = ((0, 0, 255), (240, 0, 159), (0, 165, 255), (255, 255, 0),(255, 0, 255), (255,255,255))
    refObj_red = None
    refObj_green = None
    box = None
    box_green = None
    box_red = None
    

    testt = frame.copy()

    # only proceed if at least one contour was found
    if len(cnts_green) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts_green, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    
        # only proceed if the radius meets a minimum size. Correct this value for your obect's size
        if radius > 0.5:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(testt, (int(x), int(y)), int(radius), colors["green"], 2)
            cv2.putText(testt,"green ball", (int(x-radius),int(y-radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors["green"],2)
    

    for c in cnts_green:
        # if the contour is not sufficiently large, ignore it
        if cv2.contourArea(c) < 100:
            continue
        # compute the rotated bounding box of the contour
        box_green = cv2.minAreaRect(c)
        box_green = cv2.cv.BoxPoints(box_green) if imutils.is_cv2() else cv2.boxPoints(box_green)
        box_green = np.array(box_green, dtype="int")
        # order the points in the contour such that they appear
        # in top-left, top-right, bottom-right, and bottom-left
        # order, then draw the outline of the rotated bounding
        # box
        if refObj_green is None :
            box_green = perspective.order_points(box_green)
            # compute the center of the bounding box
            cX_green = np.average(box_green[:, 0])
            cY_green = np.average(box_green[:, 1])

            # if this is the first contour we are examining (i.e.,
            # the left-most contour), we presume this is the
            # reference object
            (tl, tr, br, bl) = box_green
            (tlblX, tlblY) = midpoint(tl, bl)
            (trbrX, trbrY) = midpoint(tr, br)
            # compute the Euclidean distance between the midpoints,
            # then construct the reference object
            D = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
            refObj_green = (box_green, (cX_green, cY_green), D / width)
            continue

        # draw the contours on the image
        cv2.drawContours(testt, [box_green.astype("int")], -1, (0, 255, 0), 2)
        cv2.drawContours(testt, [refObj_green[0].astype("int")], -1, (0, 255, 0), 2)
        

    if len(cnts_red) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts_red, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    
        # only proceed if the radius meets a minimum size. Correct this value for your obect's size
        if radius > 0.5:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(testt, (int(x), int(y)), int(radius), colors["red"], 2)
            cv2.putText(testt,"Red ball", (int(x-radius),int(y-radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors["red"],2)
    

    for c in cnts_red:
        # if the contour is not sufficiently large, ignore it
        if cv2.contourArea(c) < 100:
            continue
        # compute the rotated bounding box of the contour
        box_red = cv2.minAreaRect(c)
        box_red = cv2.cv.BoxPoints(box_red) if imutils.is_cv2() else cv2.boxPoints(box_red)
        box_red = np.array(box_red, dtype="int")
        # order the points in the contour such that they appear
        # in top-left, top-right, bottom-right, and bottom-left
        # order, then draw the outline of the rotated bounding
        # box
        box_red = perspective.order_points(box_red)
        # compute the center of the bounding box
        cX_red = np.average(box_red[:, 0])
        cY_red = np.average(box_red[:, 1])

        if refObj_red is None:
            # if this is the first contour we are examining (i.e.,
            # the left-most contour), we presume this is the
            # reference object
            (tlr, trr, brr, blr) = box_red
            (tlrblrX, tlrblrY) = midpoint(tlr, blr)
            (trrbrrX, trrbrrY) = midpoint(trr, brr)
            # compute the Euclidean distance between the midpoints,
            # then constrruct the reference object
            D = dist.euclidean((tlrblrX, tlrblrY), (trrbrrX, trrbrrY))
            refObj_red = (box_red, (cX_red, cY_red), D / width)
            continue
        
        # draw the contours on the image
        cv2.drawContours(testt, [box_red.astype("int")], -1, (0, 255, 0), 2)
        cv2.drawContours(testt, [refObj_red[0].astype("int")], -1, (0, 255, 0), 2)

    # print("red: ")
    # print(refObj_red)


    # print("green: ")
    # print(refObj_green)
    
    if refObj_green != None and refObj_red != None and box_green.any() and box_red.any():
        # stack the reference coordinates and the object coordinates
        # to include the object center
        refCoords_red = np.vstack([refObj_red[0], refObj_red[1]])
        refCoords_green = np.vstack([refObj_green[0], refObj_green[1]])
        objCoords_red = np.vstack([box_red, (cX_red, cY_red)])
        objCoords_green = np.vstack([box_green, (cX_green, cY_green)])
 
        # coordinates for center point of bounding box
        xA = refCoords_red[4][0]
        yA = refCoords_red[4][1]
        xB = objCoords_green[4][0]
        yB = objCoords_green[4][1]
        color = col[5]

        # draw circles corresponding to the current points and connect them with a line
        # cv2.circle(testt, (int(xA), int(yA)), 5, color, -1)
        # cv2.circle(testt, (int(xB), int(yB)), 5, color, -1)
        cv2.line(testt, (int(xA), int(yA)), (int(xB), int(yB)), color, 2)

        # compute the Euclidean distance between the coordinates,
        # and then convert the distance in pixels to distance in units
        D = dist.euclidean((xA, yA), (xB, yB)) / refObj_green[2]
        (mX, mY) = midpoint((xA, yA), (xB, yB))
        # print("X Tengah jarak 2 bola : ")
        # print(mX)

        # print("Distance : ")
        # print(D * 2.54)
        nilaitengah = str(mX)
        # print("Middle Value : ")
        # midval = D / 2
        # print(midval * 2.54)
        #ser1.write("480")
        
        cv2.putText(testt, "{:.1f} cm".format(D * 2.54), (int(mX), int(mY - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        iA = xA
        jA = yA
        iB = xB
        jB = yB
        # width = camera.get(3)
        # height = camera.get(4)
        # print("X Tengah layar : ")
        # print(camera.get(3)/2)
        # print(camera.get(4))
        # pov_camera = cv2.line(testt, (int(camera.get(3)/2), int(camera.get(4))), (int(iA/2) + int(iB/2),int(jA/2) + int(jB/2)), color, 2)
        # horizontal_line = cv2.line(testt, (int(camera.get(3)/2), int(0)), (int(camera.get(3)/2), int(camera.get(4))), col[1], 2)
        # cv2.line(testt, (int(0), int(camera.get(4)/2)), (int(camera.get(3)), int(camera.get(4)/2)), col[1], 2)
        # Menggambar garis pov_camera
        cv2.line(testt, (int(camera.get(3)/2), int(camera.get(4))), (int(iA/2) + int(iB/2),int(jA/2) + int(jB/2)), color, 2)

        # Menggambar garis horizontal_line
        horizontal_line = cv2.line(testt, (int(camera.get(3)/2), int(0)), (int(camera.get(3)/2), int(camera.get(4))), col[1], 2)

        
        dot_pov_camera = cv2.circle(testt, (int(iA/2) + int(iB/2), int(jA/2) + int(jB/2)), 5, color, -1)
        pointsList.append([int(iA/2) + int(iB/2), int(jA/2) + int(jB/2)])
        
        top_dot_horizontal_line = cv2.circle(testt, (int(camera.get(3)/2), int(0)), 5, col[1], -1)
        pointsList.append([int(camera.get(3)/2), int(0)])

        bottom_dot_horizontal_line = cv2.circle(testt, (int(camera.get(3)/2), int(camera.get(4))), 5, col[1], -1)
        pointsList.append([int(camera.get(3)/2), int(camera.get(4))])
        
        

        # def gradient(dot_pov_camera, bottom_dot_horizontal_line):
        #      return (bottom_dot_horizontal_line[1]-dot_pov_camera[1])/(dot_pov_camera[0]-bottom_dot_horizontal_line[0])
        

        #hampir bener tapi mentok di 90 trs sudutnya cuma setengah dari sudut asli di dunia nyata
        def getAngle():
            dot_pov_camera, bottom_dot_horizontal_line, top_dot_horizontal_line = pointsList[-3:] 
            # m1=gradient(dot_pov_camera,bottom_dot_horizontal_line)
            # m2=gradient(dot_pov_camera,top_dot_horizontal_line)
            # angR=math.atan((m2-m1)/(1+(m2*m1)))
            # angD=round(math.degrees(angR))
            m1=(((int(iA/2) + int(iB/2))-int(camera.get(3)/2))/((int(jA/2) + int(jB/2))-int(camera.get(4))))
            srad=math.atan(m1)
            sdeg=math.degrees(srad)

            # cv2.putText(testt,str(angD),(dot_pov_camera[0]-40,dot_pov_camera[1]-20),cv2.FONT_HERSHEY_COMPLEX, 1.5, (0,0,255),2) 
            # print(sdeg) 

            if sdeg < 0:
                # print("moves left")
                # print(sdeg)
                ser1.write(sdeg)

            elif sdeg > 0:
                # print("moves right")
                # print(sdeg)
                ser1.write(sdeg)

            else:
                print("No need to move horizontally")          
        
        if len(pointsList) % 3 == 0 : 
            getAngle()
        

         # Calculate deviation of the center line from the center of the frame
        # x_center = int((xA + xB) / 2)
        # y_center = int((yA + yB) / 2)
        # x_deviation = abs(x_center - int(camera.get(3) / 2))
        # y_deviation = abs(y_center - int(camera.get(4) / 2))

        # Print the deviation values
        # print("X Deviation: ", x_deviation)
        # print("Y Deviation: ", y_deviation)

        # angR = math.atan((pov_camera-horizontal_line/(1+pov_camera*horizontal_line)))
        # angD = round(math.degrees(angR))
        # print("Angle: {:.2f} degrees".format(angD))



    #     angle_rad = np.arctan2(yB - yA, xB - xA)
    #     angle_deg = np.degrees(angle_rad)

    # # Calculate the angle between the center line and the frame's horizontal line
    #     center_line_angle_rad = np.arctan2((y_center - int(camera.get(4) / 2))-(x_center - int(camera.get(3) / 2)), 1-((y_center - int(camera.get(4) / 2))*(x_center - int(camera.get(3) / 2))))
    #     center_line_angle_deg = np.degrees(center_line_angle_rad)

    # # Calculate the relative angle between the two angles
    #     relative_angle_deg = center_line_angle_deg - angle_deg

    #     print("Angle: {:.2f} degrees".format(relative_angle_deg))

    #     curvature = 1 / (2 * np.tan(np.radians(relative_angle_deg / 2)))

    #     # Create points for the curved line
    #     curve_points = []
    #     for x in range(int(camera.get(3) / 2), int((xA + xB) / 2)):
    #         y = int(curvature * (x - int(camera.get(3) / 2)) ** 2) + int(camera.get(4) / 2)
    #         curve_points.append((x, y))

    #     # Draw the curved line on the frame
    #     for i in range(len(curve_points) - 1):
    #         cv2.line(testt, curve_points[i], curve_points[i + 1], (255, 255, 255), 2)

        # Calculate the bearing angle
    #     dx = xB - xA
    #     dy = yB - yA
    #     bearing_rad = np.arctan2(dy, dx)
    #     bearing_deg = np.degrees(bearing_rad)
        
    #     # Check the bearing angle and move the camera accordingly
    #     if bearing_deg > 0:
    #         print("Move camera to the right")
    #         print("Bearing Angle: {:.2f} degrees".format(bearing_deg))
    #     elif bearing_deg < 0:
    #         print("Move camera to the left")
    #         print("Bearing Angle: {:.2f} degrees".format(bearing_deg))
    #     else:
    #         print("Camera aligned with the balls")

    # else:
    #     nilaitengah = nilaitengah


    #     # derajat biasa
    #     angle_rad = np.arctan2(yB - yA, xB - xA)
    #     angle_deg = np.degrees(angle_rad)

    #     # Calculate the angle between the center line and the frame's horizontal line
    #     center_line_angle_rad = np.arctan2(y_center - int(camera.get(4) / 2), x_center - int(camera.get(3) / 2))
    #     center_line_angle_deg = np.degrees(center_line_angle_rad)

    #     # Calculate the relative angle between the two angles
    #     relative_angle_deg = center_line_angle_deg - angle_deg

    # # Print the relative angle
    #     if relative_angle_deg < 0:
    #         print("moves right")
    #         print("Angle: {:.2f} degrees".format(-relative_angle_deg))
    #     elif relative_angle_deg > 0:
    #         print("moves left")
    #         print("Angle: {:.2f} degrees".format(relative_angle_deg))
    #     else:
    #         print("No need to move horizontally")

    # else:
    #     nilaitengah = nilaitengah

        # algoritma awal dengan nilai tolerance
        # Check if both X and Y values are less than the tolerance value
    #     if x_deviation < tolerance_value and y_deviation < tolerance_value:
    #         print("continue to move and track the object.")
    #     else:
    #         # Move the robot to minimize the deviation
    #         if x_deviation > tolerance_value and y_deviation < tolerance_value:
    #             # Move left or right to minimize X
    #             if x_center < int(camera.get(3) / 2):
    #                 print("moves left")
    #             else:
    #                 print("moves right")
    #         elif x_deviation < tolerance_value and y_deviation > tolerance_value:
    #             # Move forward or backward to minimize Y
    #             if y_center < int(camera.get(4) / 2):
    #                 print("moves forward")
    #         else:
    #             # Both X and Y are greater than the tolerance value, move in the direction with the maximum deviation
    #             if x_deviation > y_deviation:
    #                 if x_center < int(camera.get(3) / 2):
    #                     print("moves left")
    #                 else:
    #                     print("moves right")
    #             else:
    #                 if y_center < int(camera.get(4) / 2):
    #                     print("moves forward")

    # else:
    #     nilaitengah = nilaitengah


    # show the frame to our screen
    # cv2.imshow("Frame", frame)
    # show the test to our screen
    # testt.set(3 , 1080  ) # width        
    # testt.set(4 , 720  ) # height 
    # print("Nilai Tengah : " + nilaitengah)
    # ser1.write(nilaitengah.encode())
    # ser1.write('\n'.encode())
    cv2.imshow("Frame", testt)
    # results_red = cv2.resize(result_red, (480,270))
    # cv2.imshow("Mask Red", results_red)
    # results_green = cv2.resize(result_green, (480,270))
    # cv2.imshow("Mask Green", results_green)
    
    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
 
# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()