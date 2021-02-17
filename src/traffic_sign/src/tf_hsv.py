import cv2

image = cv2.imread("green.jpg")
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(hsv)

cv2.imshow("h", h)
cv2.imshow("s", s)
cv2.imshow("v", v)
cv2.waitKey(0)
cv2.destroyAllWindows()

