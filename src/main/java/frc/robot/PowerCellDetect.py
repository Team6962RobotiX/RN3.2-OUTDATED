def find_circles(image):
    H_low = 20
    H_high = 45
    S_low = 145
    S_high = 255
    V_low = 115
    V_high = 255

    result = image.copy()
    
    height, width, channels = image.shape
    scale = math.sqrt(width * height)
    blur_size = round(scale / 85) * 2 + 1
    kernel_size = round(scale / 115)
    circle_spacing = round(scale / 12)

    blur = cv2.GaussianBlur(image, (blur_size, blur_size), 1)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    lower_hsv = np.array([H_low, S_low, V_low])
    upper_hsv = np.array([H_high, S_high, V_high])
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    open_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, None, iterations=8)
    canny = cv2.Canny(open_mask, 0, 255)
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    closed_canny = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)
    
    contour_img = np.zeros((height, width, 1), np.uint8)

    contours, hierarchy = cv2.findContours(closed_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    if len(contours) != 0:
        hull_list = []
        for i in range(len(contours)):
            hull = cv2.convexHull(contours[i])
            x, y, w, h = cv2.boundingRect(hull)
            r = (w + h) / 4
            if ((math.pi * r ** 2) * 0.9 < cv2.contourArea(hull)) & ((math.pi * r ** 2) * 1.1 > cv2.contourArea(hull)):
                if (w * 0.9 < h) & (w * 1.1 > h):
                    if ((2 * math.pi * r) * 0.9 < cv2.arcLength(hull, True)) & ((2 * math.pi * r) * 1.1 > cv2.arcLength(hull, True)):
                        hull_list.append(hull)

        cv2.drawContours(contour_img, hull_list, -1, (255, 100, 75), -1)

    circles = cv2.HoughCircles(contour_img, cv2.HOUGH_GRADIENT, 2.9, circle_spacing)
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        cv2.putText(resize, str(len(circles)) + " circles", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
        for (x, y, r) in circles:
            cv2.circle(result, (x, y), r, (255, 100, 75), 4)
            cv2.circle(result, (x, y), 3, (255, 100, 75), 5)

    cv2.imshow("result", result)
    
    return circles
