# Returns 2D array of circles, [[x, y, r], [x, y, r], ...]
def find_circles(image):
    H_low = 20
    H_high = 45
    S_low = 145
    S_high = 255
    V_low = 115
    V_high = 255

    blur_size = 13
    kernel_size = 5
    circle_thresh = 0.12

    result = image.copy()
    height, width, channels = image.shape
    circles = []

    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    largeKernel = np.ones((kernel_size * 2, kernel_size * 2), np.uint8)

    blur = cv2.GaussianBlur(image, (blur_size, blur_size), 1)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    lower_hsv = np.array([H_low, S_low, V_low])
    upper_hsv = np.array([H_high, S_high, V_high])
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    open_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, largeKernel)

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
            for j in range(len(hull)):
                dist = math.sqrt(((x + w / 2) - hull[j][0][0]) ** 2 + ((y + h / 2) - hull[j][0][1]) ** 2)
                if (dist * (1 - circle_thresh) > r) or (dist * (1 + circle_thresh) < r):
                    isCircle = False
            if isCircle:
                circles.append([int((x + w / 2)), int((y + h / 2)), int(r)])
                hull_list.append(hull)

    # Displaying:
    # for circle in circles:
        # cv2.circle(result, (circle[0], circle[1]), circle[2], (255, 100, 75), int(circle[2] / 15))
        # cv2.circle(result, (circle[0], circle[1]), 1, (255, 100, 75), int(circle[2] / 15))
    # cv2.imshow("result", result)

    return circles
