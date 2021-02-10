package frc.robot;
import org.opencv.core.*;
import org.opencv.imgproc.*;
import java.util.*;

public class PowerCellDetect {
	private static final int H_LOW = 20;
	private static final int H_HIGH = 45;
	private static final int S_LOW = 145;
	private static final int S_HIGH = 255;
	private static final int V_LOW = 115;
	private static final int V_HIGH = 255;
	private static final int BLUR_SIZE = 5;
	private static final int KERNEL_SIZE = 5;
	private static final int CIRCLE_SPACING = 50;

	public static Mat detect(Mat input) {
		if(input == null || input.empty()) { return null; }

		Mat result = new Mat();
		input.copyTo(result);
		
		int height = input.height();
		int width = input.width();

		Mat blurred = new Mat();
		Imgproc.GaussianBlur(input, blurred, new Size(BLUR_SIZE, BLUR_SIZE), 1);
		Mat hsv = new Mat();
		Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_BGR2HSV);
		Scalar lower_hsv = new Scalar(H_LOW, S_LOW, V_LOW);
		Scalar upper_hsv = new Scalar(H_HIGH, S_HIGH, V_HIGH);
		Mat mask = new Mat();
		Core.inRange(hsv, lower_hsv, upper_hsv, mask);
		Mat openMask = new Mat();
		Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
		for (int i = 0; i < 8; i++) {
			if (i == 0) {
				Imgproc.morphologyEx(mask, openMask, Imgproc.MORPH_OPEN, element);
			} else {
				Imgproc.morphologyEx(openMask, openMask, Imgproc.MORPH_OPEN, element);
			}
		}
		Mat canny = new Mat();
		Imgproc.Canny(openMask, canny, 0, 255);
		element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(KERNEL_SIZE, KERNEL_SIZE));
		Mat closedCanny = new Mat();
		Imgproc.morphologyEx(canny, closedCanny, Imgproc.MORPH_CLOSE, element);
		System.out.println(height);
		System.out.println(width);
		Mat contourImg = new Mat(height, width, CvType.CV_8UC1);
		Mat hierarchy = new Mat();
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		Imgproc.findContours(closedCanny, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
		if (contours.size() != 0) {
			List<MatOfPoint> hullList = new ArrayList<MatOfPoint>();
			for (MatOfPoint contour : contours) {
				List<Point> list = new ArrayList<Point>();
				MatOfPoint hullPoints = new MatOfPoint();
				MatOfInt hull = new MatOfInt();
				Imgproc.convexHull(contour, hull);
				for(int i=0;i<hull.toArray().length-1;i+=2) {
					list.add(new Point(hull.toArray()[i], hull.toArray()[i+1]));
				}
				hullPoints.fromList(list);
				Rect rect = Imgproc.boundingRect(hullPoints);
				double r = (rect.width + rect.height) / 4.0;
				if ((Math.pow(Math.PI * r, 2)) * 0.9 < Imgproc.contourArea(hullPoints) && ((Math.pow(Math.PI * r, 2)) * 1.1 > Imgproc.contourArea(hullPoints))) {
					if ((rect.width * 0.9 < rect.height) && (rect.width * 1.1 > rect.height)) {
						if (((2 * Math.PI * r) * 0.9 < Imgproc.arcLength(new MatOfPoint2f(hullPoints), true)) && ((2 * Math.PI * r) * 1.1 > Imgproc.arcLength(new MatOfPoint2f(hullPoints), true))) {
							hullList.add(hullPoints);
						}
					}
				}
			}
			Imgproc.drawContours(contourImg, hullList, -1, new Scalar(255, 255, 255), -1);
		}
		Mat circles = new Mat();
		Imgproc.HoughCircles(contourImg, circles, Imgproc.HOUGH_GRADIENT, 3.5, CIRCLE_SPACING);
		if (!circles.empty()) {
			for (int i = 0; i < circles.cols(); i++) {
				double[] xyr = circles.get(0, i);
				Point center = new Point((int)xyr[0], (int)xyr[1]);
				Imgproc.circle(result, center, (int)xyr[2], new Scalar(255, 100, 75), 4);
				Imgproc.circle(result, center, 3, new Scalar(255, 100, 75), 5);
			}
		}
		return contourImg;
	}
}
