import cv2
import numpy as np

path1 = "C:/Users/puigm/Desktop/ROS_github/rUBot_mecanum_ws/src/rubot_projects/photos/"
        
img_init = cv2.imread(path1+'image_left_test.png')
img=img_init.copy()

print("Img size: "+str(img.shape))
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

ret,thresh = cv2.threshold(gray,80,255,0)
#contours,hierarchy = cv2.findContours(thresh, 1, 2)
contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
print("Number of contours detected:", len(contours))

for cnt in contours:
   x1,y1 = cnt[0][0]
   approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
   #print ("Approx: "+ str(len(approx)))
   if len(approx) == 4:
      x, y, w, h = cv2.boundingRect(cnt)
      print("x: "+str(x)+ " y: "+str(y)+" w: "+str(w)+" h: "+str(h))
      ratio = float(w)/h
      if ratio >= 0.9 and ratio <= 1.1:
         img2 = cv2.drawContours(img, [cnt], -1, (0,255,255), 3)
         cv2.putText(img2, 'Square', (x1-100, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
         img3 = img_init[y:y+h,x:x+w]
      else:
          img2 = cv2.drawContours(img, [cnt], -1, (0,255,0), 3)
          cv2.putText(img2, 'Rectangle', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
         

cv2.imshow("Init", img_init)
cv2.imshow("Shapes", img)
cv2.imshow("Detected sign", img3)
cv2.imshow("gray", gray)
cv2.imshow("thresh", thresh)
# Resize image and save
sign_dim = (60,60)
sign = cv2.resize(img3,sign_dim)
cv2.imwrite(path1+"image_left_test2.png",sign)
cv2.waitKey(0)
cv2.destroyAllWindows()