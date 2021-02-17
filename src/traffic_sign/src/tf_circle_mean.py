import cv2
import numpy as np

image = cv2.imread("green.jpg")
image = cv2.medianBlur(image,5)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(hsv)

circles = cv2.HoughCircles(v, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=25, minRadius=0, maxRadius=30)

circles = np.uint16(np.around(circles))

for i in circles[0,:]:
    cv2.circle(image,(i[0],i[1]),i[2],(0,255,0),2)
    cr_img = v[i[1]-10 : i[1]+10, i[0]-10 : i[0]+10]
    img_str = 'x: {0}, y: {1}, mean : {2}'.format(i[0],i[1], cr_img.mean())
    print(img_str)

cv2.imshow('img', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
