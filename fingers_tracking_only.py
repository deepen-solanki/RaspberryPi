#Use HSV_thresholder file to segment each fingertip color appropriately. Find lowH,lowS,lowV and highH,highS,highV such that the fingertip is the only thing in the frame
#fill those was values in the function self.DetectObject(frame_HSV,lowH,lowS,lowV,highH,highS,highV)
#Based on the code from https://github.com/SriramEmarose/Motion-Prediction-with-Kalman-Filter/blob/master/KalmanFilter.py
#This version only performs tracking.
#Next upload will send data to Arduino Uno and move a robotic hand


import cv2 as cv
import numpy as np
import sys
import time

class ProcessImage:

    def DetectObject(self):

        vid = cv.VideoCapture(0)

        if(vid.isOpened() == False):
            print('Cannot open input video')
            return

        while(vid.isOpened()):
            rc, frame = vid.read()
            frame1 = frame.copy()

            if(rc == True):
  
                [refX, refY] = self.contourArea(frame,frame1)

                frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

                [pinkyX, pinkyY] = self.DetectObject(frame_HSV, 153, 76, 0, 167, 246, 255)
                
                [ringX, ringY] = self.DetectObject(frame_HSV, 62, 20, 78, 93, 116, 147)
                
                [middleX, middleY] = self.DetectObject(frame_HSV, 172, 137, 89, 180, 197, 186)

                [indexX, indexY] = self.DetectObject(frame_HSV, 88, 102, 61, 122, 165, 160)

                [thumbX, thumbY] = self.DetectObject(frame, 10, 68, 175, 24, 154, 213)
                
                #Pinky Actual
                cv.circle(frame1, (int(pinkyX), int(pinkyY)), 2, [0,0,255], 2, 8)
                cv.putText(frame1, "Pinky", (int(pinkyX + 50), int(pinkyY + 20)), cv.FONT_HERSHEY_SIMPLEX,0.5, [50,200,250])
                

                #Ring Actual  
                cv.circle(frame1, (int(ringX), int(ringY)), 2, [0,0,255], 2, 8)
                cv.putText(frame1, "Ring", (int(ringX + 50), int(ringY + 20)), cv.FONT_HERSHEY_SIMPLEX,0.5, [50,200,250])
    

                #Middle Actual
                cv.circle(frame1, (int(middleX), int(middleY)), 2, [0,0,255], 2, 8)
                cv.putText(frame1, "Middle", (int(middleX + 50), int(middleY + 20)), cv.FONT_HERSHEY_SIMPLEX,0.5, [50,200,250])
                

                #Index Actual
                cv.circle(frame1, (int(indexX), int(indexY)), 2, [0,0,255], 2, 8)
                cv.putText(frame1, "Index", (int(indexX + 50), int(indexY + 20)), cv.FONT_HERSHEY_SIMPLEX,0.5, [50,200,250])
                

                #Thumb Actual
                cv.circle(frame1, (int(thumbX), int(thumbY)), 2, [0,0,255], 2, 8)
                cv.putText(frame1, "Thumb", (int(thumbX + 50), int(thumbY + 20)), cv.FONT_HERSHEY_SIMPLEX,0.5, [50,200,250])
                
                cv.imshow('Input', frame1)

                if (cv.waitKey(1) & 0xFF == ord('q')):
                    break

            else:
                break

        vid.release()
        cv.destroyAllWindows()

    def DetectObject(self, frame_HSV, loH, loS, loV, hiH, hiS, hiV):

        #Apply mask
        greenMask = cv.inRange(frame_HSV,(loH, loS, loV),(hiH, hiS, hiV)) #This is the line being tested
        
        # Dilate
        kernel = np.ones((5, 5), np.uint8)
        greenMaskDilated = cv.dilate(greenMask, kernel)

        # Find object as it is the biggest blob in the frame
        [nLabels, labels, stats, centroids] = cv.connectedComponentsWithStats(greenMaskDilated, 8, cv.CV_32S)

        # First biggest contour is image border always, Remove it
        stats = np.delete(stats, (0), axis = 0)
        try:
            maxBlobIdx_i, maxBlobIdx_j = np.unravel_index(stats.argmax(), stats.shape)

        # This is our ball coords that needs to be tracked
            objX = stats[maxBlobIdx_i, 0] + (stats[maxBlobIdx_i, 2]/2)
            objY = stats[maxBlobIdx_i, 1] + (stats[maxBlobIdx_i, 3]/2)
            return [objX, objY]
        except:
               pass

        return [0,0]

    def contourArea(self, img, img2):
        
        grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        value = (31, 31)
        blurred = cv.GaussianBlur(grey, value, 0)
        retVal,thresh = cv.threshold(blurred,60,255,cv.THRESH_BINARY_INV)
        
        contours, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        highestY = -100000
        highMostContour = 0
        
        if len(contours)!=0:
            
            for contour in contours:
                (x,y),radius = cv.minEnclosingCircle(contour)
                center = (int(x),int(y))
                radius = int(radius)
                
                if int(y) > highestY:
                    highestY = int(y)
                    highMostContour = contour
            
            if len(highMostContour) != 0:
                cv.drawContours(img2, highMostContour, -1, (255, 255, 255), 3) 
                handContour = highMostContour
                hullHandContour = cv.convexHull(handContour, returnPoints = False)
                handMoments = cv.moments(handContour)
                if handMoments["m00"] != 0:
                    handXCenterMoment = int(handMoments["m10"]/handMoments["m00"])
                    handYCenterMoment = int(handMoments["m01"]/handMoments["m00"])               
                    
                    centroidX = handXCenterMoment
                    centroidY = handYCenterMoment + 10 
                    
                    palmRadius = cv.pointPolygonTest(handContour,(centroidX,handYCenterMoment), True)
                    print("palmRadius: ",palmRadius)
                    
                    if palmRadius > 0:
                        cv.circle(img2, (handXCenterMoment, handYCenterMoment), int(palmRadius), (255, 255, 255), 2)
                    
                    hullPoints = []
                    for i in hullHandContour:
                        hullPoints.append(handContour[i[0]])
                    hullPoints = np.array(hullPoints, dtype = np.int32)

                    return [centroidX, centroidY]
            
                else:
                    return [0,0]
            else:
                return [0,0]
        else:
            return [0,0]
        

#Main Function
def main():

    processImg = ProcessImage()
    processImg.DetectObject()


if __name__ == "__main__":
    main()

