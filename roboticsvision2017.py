import numpy as np
import cv2
import math

from networktables import NetworkTables

NetworkTables.initialize(server='roborio-3929-frc.local')
table = NetworkTables.getTable('VisionTable')

cap = cv2.VideoCapture()
cap.open("http://10.39.29.105/mjpg/video.mjpg")

#Measured in px
frameWidth = cap.get(3)
frameHeight = cap.get(4)

capturing = True
logic = True

#Measured in mm
targetLength = 2 #2 in
totalLength = 10.25 #10.25 in

#Measured in px
focalLength = 750

#20 in away, 2 in wide tape

def main():
    offset = "def"
    distance = 10
    found = False
    
    contours = []
    contoursApprox = []
    contoursAreas = []
    contoursAreasSorted = []
    target1 = 0
    target2 = 0

    ###CAPTURE AND GUI###

    # Capture

    ret, frame = cap.read()

    # Set frame dimensions
    
    ret = cap.set(3, frameWidth)
    ret = cap.set(4, frameHeight)

    # Process frame
        # Convert frame from BGR to HSV
        
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define threshold

            #actual value from color drop : 179, 100, 100
    threshLower = np.array([50,50,240])
    threshUpper = np.array([100,255,255])
    
        # Create mask from frame
      
        # Create mask from frame
    
    mask = cv2.inRange(hsv, threshLower, threshUpper)

        # Define contours
        
    contoursRaw, hierarchy= cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for i in range(0,len(contoursRaw)):
        contourTest = contoursRaw[i]
        extremaLeft = tuple(contourTest[contourTest[:, :, 0].argmin()][0])
        extremaRight = tuple(contourTest[contourTest[:, :, 0].argmax()][0])
        extremaUp = tuple(contourTest[contourTest[:, :, 1].argmin()][0])
        extremaDown = tuple(contourTest[contourTest[:, :, 1].argmax()][0])
        length = extremaRight[0] - extremaLeft[0]
        height = extremaDown[1] - extremaUp[1]
        if long (length) != 0:
            if abs(long (height)/ long (length) - 2.5) < 2 and cv2.contourArea(contourTest) > 100:
                #if extremaLeft[0] > 100:
                contours.append(contoursRaw[i])
        # Find largest contour in terms of area

    for i in range(0,len(contours)):
        cnt = contours[i]
        contoursAreas.append(cv2.contourArea(contours[i]))
        contoursAreasSorted.append(cv2.contourArea(contours[i]))
        contoursAreasSorted.sort()
    for i in range(0,len(contours)):
        if contoursAreas[i] == contoursAreasSorted[len(contoursAreas) - 1]:
            target1 = i
        elif contoursAreas[i] == contoursAreasSorted[len(contoursAreas) - 2]:
            target2 = i

        # Fill in contours

    if len(contours) >= 2:
        targetContours = [contours[target1] , contours[target2]]
        for i in range(0, len(targetContours)):
            peri = cv2.arcLength(targetContours[i], True)
            approx = cv2.approxPolyDP(targetContours[i], 0.04 * peri, True)
            contoursApprox.append(approx)
        cv2.fillPoly(mask, targetContours, color=(200,200,200))
        cv2.fillPoly(mask, contoursApprox, color=(255,255,255))

    # Display frame - unnused

    ##cv2.imshow('camera',frame)

        # Display frame with mask and contours

    cv2.imshow('mask',mask)

    # Closes frame upon pressing key "esc"

    if logic == True:

        targetContourA = []
        targetContourB = []

        ###LOGIC###

        if len(contours) >= 2:
            
            targetContour1 = targetContours[0]
            targetContour2 = targetContours[1]

                #Sorts targets

            if tuple(targetContour1[targetContour1[:, :, 0].argmax()][0]) < tuple(targetContour2[targetContour2[:, :, 0].argmin()][0]):
                targetContourA = targetContours[0]
                targetContourB = targetContours[1]
                targetApproxA = contoursApprox[0]
                targetApproxB = contoursApprox[1]
            elif tuple(targetContour2[targetContour2[:, :, 0].argmax()][0]) < tuple(targetContour1[targetContour1[:, :, 0].argmin()][0]):
                targetContourA = targetContours[1]
                targetContourB = targetContours[0]
                targetApproxA = contoursApprox[1]
                targetApproxB = contoursApprox[0]

            if targetContourA != [] and targetContourB != []:
            
                #Finds extrema

                extremaLeftA = tuple(targetApproxA[targetApproxA[:, :, 0].argmin()][0])
                extremaRightA = tuple(targetApproxA[targetApproxA[:, :, 0].argmax()][0])
                extremaUpA = tuple(targetApproxA[targetApproxA[:, :, 1].argmin()][0])
                extremaDownA = tuple(targetApproxA[targetApproxA[:, :, 1].argmax()][0])
                
                extremaLeftB = tuple(targetApproxB[targetApproxB[:, :, 0].argmin()][0])
                extremaRightB = tuple(targetApproxB[targetApproxB[:, :, 0].argmax()][0])
                extremaUpB = tuple(targetApproxB[targetApproxB[:, :, 1].argmin()][0])
                extremaDownB = tuple(targetApproxB[targetApproxB[:, :, 1].argmax()][0])

                #Finds length, height, and ratios

                targetALength = extremaRightA[0] - extremaLeftA[0]
                targetAHeight = extremaDownA[1] - extremaUpA[1]
                
                targetBLength = extremaRightB[0] - extremaLeftB[0]
                targetBHeight = extremaDownB[1] - extremaUpB[1]

                if long (targetALength) !=0 and long (targetBLength) != 0:
                    targetARatio = long (targetAHeight) / long (targetALength)
                    targetBRatio = long (targetBHeight) / long (targetBLength)

                #The logics

                targetDistance = (focalLength * totalLength) / (extremaRightB[0] - extremaLeftA[0])
                centerTolerance = ((focalLength * targetLength) / targetDistance) * 2
                
                targetPosition = "found"
                found = True
                if abs(((extremaRightB[0] + extremaLeftA[0])/ 2) - frameWidth / 2)  <=  centerTolerance:
                    targetPosition = "centered"
                    offset = "centered"
                elif extremaLeftA[0] < frameWidth - extremaRightB[0]:
                    offset = "left"
                elif extremaLeftA[0] > frameWidth - extremaRightB[0]:
                    offset = "right"
                else:
                    offset = "def"

                table.putNumber('distance', targetDistance)
                table.putString('offset', offset)
        else:
            targetPosition = "missing"
            found = False

        #print found
        table.putBoolean('found', found)
        
while capturing == True:
    main()
    if cv2.waitKey(10) == 27:
        capturing = False

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

