import numpy as np
import cv2
import math

cap = cv2.VideoCapture(0)


def callback(x):
    global H_low, H_high, S_low, S_high, V_low, V_high, kernel_size, circle_thresh
    H_low = cv2.getTrackbarPos('low H', 'controls')
    H_high = cv2.getTrackbarPos('high H', 'controls')
    S_low = cv2.getTrackbarPos('low S', 'controls')
    S_high = cv2.getTrackbarPos('high S', 'controls')
    V_low = cv2.getTrackbarPos('low V', 'controls')
    V_high = cv2.getTrackbarPos('high V', 'controls')

    kernel_size = cv2.getTrackbarPos('Kernel Size', 'controls')
    circle_thresh = cv2.getTrackbarPos('Circle Threshold', 'controls') / 100


cv2.namedWindow('controls', 2)
cv2.resizeWindow("controls", 550, 10);

H_low = 20
H_high = 45
S_low = 110
S_high = 255
V_low = 80
V_high = 255

blur_size = 5
kernel_size = 5
circle_thresh = 0.12

cv2.createTrackbar('low H', 'controls', H_low, 179, callback)
cv2.createTrackbar('high H', 'controls', H_high, 179, callback)
cv2.createTrackbar('low S', 'controls', S_low, 255, callback)
cv2.createTrackbar('high S', 'controls', S_high, 255, callback)
cv2.createTrackbar('low V', 'controls', V_low, 255, callback)
cv2.createTrackbar('high V', 'controls', V_high, 255, callback)

cv2.createTrackbar('Kernel Size', 'controls', kernel_size, 16, callback)
cv2.createTrackbar('Circle Threshold', 'controls', int(circle_thresh * 100), 99, callback)


def find_circles(image):
    result = image.copy()
    height, width, channels = image.shape
    circles = []

    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    kernel2x = np.ones((kernel_size * 2, kernel_size * 2), np.uint8)

    blur = cv2.GaussianBlur(image, (blur_size, blur_size), 1)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    lower_hsv = np.array([H_low, S_low, V_low])
    upper_hsv = np.array([H_high, S_high, V_high])
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    open_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel2x)
    colored_mask = cv2.bitwise_and(blur, blur, mask=open_mask)

    canny = cv2.Canny(open_mask, 0, 255)
    closed_canny = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)

    contours, hierarchy = cv2.findContours(closed_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) != 0:
        hull_list = []
        for i in range(len(contours)):
            hull = cv2.convexHull(contours[i])
            x, y, w, h = cv2.boundingRect(hull)
            r = (w + h) / 4
            isCircle = True
            accuracy = 0
            for j in range(len(hull)):
                dist = math.sqrt(((x + w / 2) - hull[j][0][0]) ** 2 + ((y + h / 2) - hull[j][0][1]) ** 2)
                accuracy += abs(dist - r) / (r * circle_thresh)
                if (dist * (1 - circle_thresh) > r) or (dist * (1 + circle_thresh) < r):
                    isCircle = False
            accuracy /= len(hull)
            if isCircle:
                circles.append([int((x + w / 2)), int((y + h / 2)), int(r), accuracy])
                hull_list.append(hull)

    cv2.imshow("colored_mask", colored_mask)
    return circles


if cap.isOpened():
    while True:
        ret, img = cap.read()
        frame = cv2.resize(img, (640, 360))
        circle_result = find_circles(frame)
        for circle in circle_result:
            cv2.circle(frame, (circle[0], circle[1]), circle[2], (255, 100, 75), int(circle[2] / 15))
            cv2.circle(frame, (circle[0], circle[1]), 1, (255, 100, 75), int(circle[2] / 15))
            cv2.putText(frame, str(round((1 - circle[3]) * 100)) + "%", (circle[0] + 5, circle[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, circle[2] / 120, (255, 100, 75), int(circle[2] / 60), cv2.LINE_AA)
        cv2.imshow("result", frame)

        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
