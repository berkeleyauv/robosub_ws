import numpy as np
import cv2
from datetime import datetime 

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    cap.open()
# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('/media/usb/raw_images/zed_test_' + str(datetime.now()) + '.avi',fourcc, 20.0, (int(cap.get(3)),int(cap.get(4))))
print(cap.isOpened())
while(cap.isOpened()):
    ret, frame = cap.read()
    if ret==True:
        out.write(frame)
        print(frame.shape)
        break
        #cv2.imwrite('frame3.png',frame)
        #print('wrote')
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()

