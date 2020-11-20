#!/usr/bin/env python 

import cv2
import base64
video_capture = cv2.VideoCapture(0)
#video_capture = cv2.VideoCapture('video/tennis-ball-video.mp4')
fps=24
ms_to_wait=int(1000/fps)

while (True):
    ret, frame = video_capture.read()
    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.resize(frame, (0,0), fx=0.2,fy=0.2)
    ret, buffer= cv2.imencode('.jpg', frame)
    text=base64.b64encode(buffer)
    print(text)
    #print('s')
    #your object detection algorith
    #takes as input an image
    #return a list of bounding boxes of the detected object
    #cv2.line(frame,(0,0),(511,511),(255,0,0),5)
    cv2.imshow("Frame",frame)
    if cv2.waitKey(ms_to_wait) & 0xFF == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows()
