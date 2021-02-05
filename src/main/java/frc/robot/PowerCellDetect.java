import org.opencv.core.*;
import org.opencv.imgproc.*;

public class PowerCellDetect {
	private static final int H_LOW = 20;
	private static final int H_HIGH = 45;
	private static final int S_LOW = 145;
	private static final int S_HIGH = 255;
	private static final int V_LOW = 115;
	private static final int V_HIGH = 255;
	private static final int BLUR_SIZE = 15;
	private static final int KERNEL_SIZE = 5;
	private static final int CIRCLE_SPACING = 50;

	public static Mat detect(Mat input) {
		Mat result = new Mat();
		input.copyTo(result);
		
		int height = input.height();
		int width = input.width();

		Mat blurred;
		Imgproc.GaussianBlur(input, blurred, new Size(BLUR_SIZE, BLUR_SIZE), 1);
		Mat hsv;
		Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_BGR2HSV);
		Scalar lower_hsv = new Scalar(H_LOW, S_LOW, V_LOW);
		Scalar upper_hsv = new Scalar(H_HIGH, S_HIGH, V_HIGH);
		Mat mask;
		Core.inRange(hsv, lower_hsv, upper_hsv, mask);
		Mat openMask;
		for (int i = 0; i < 8; i++) {
			if (i == 0) {
				Imgproc.morphologyEx(mask, openMask, Imgproc.MORPH_OPEN,null);
			} else {
				Imgproc.morphologyEx(openMask, openMask, Imgproc.MORPH_OPEN,null);
			}
		}
		Mat canny;
		Imgproc.Canny(openMask, canny, 0, 255);
		Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(KERNEL_SIZE, KERNEL_SIZE));
		Mat closedCanny;
		Imgproc.morphologyEx(canny, closedCanny, Imgproc.MORPH_CLOSE, element);
		Mat contourImg = new Mat(height, width, CvType.CV_8UC1);
		Mat hierarchy;
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		Imgproc.findContours(closedCanny, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
		if (contours.size() != 0) {
			List<MatOfPoint> hullList = new ArrayList<MatOfPoint>();
			for (MatOfPoint contour : contours) {
				MatOfInt hull = new MatOfInt();
				Imgproc.convexHull(contour, hull);
				Rect rect = Imgproc.boundingRect(hull);
				double r = (rect.width + rect.height) / 4.0;
				if ((Math.pow(Math.PI * r, 2)) * 0.9 < Imgproc.contourArea(hull) && ((Math.pow(Math.PI * r, 2)) * 1.1 > Imgproc.contourArea(hull))) {
					if ((rect.width * 0.9 < rect.height) && (rect.width * 1.1 > rect.height)) {
						if (((2 * Math.PI * r) * 0.9 < Imgproc.arcLength(hull, true)) && ((2 * Math.PI * r) * 1.1 > Imgproc.arcLength(hull, true))) {
							hullList.add(hull);
						}
					}
				}
			}
			Imgproc.drawContours(contourImg, hullList, -1, new Scalar(255, 255, 255), -1);
		}
		Mat circles = null;
		Imgproc.HoughCircles(contourImg, Imgproc.HOUGH_GRADIENT, 2.9, CIRCLE_SPACING);
		if (circles != null) {
			for (int i = 0; i < circles.cols(); i++) {
				double[] xyr = circles.get(0, i);
				Point center = new Point((int)xyr[0], (int)xyr[1]);
				Imgproc.circle(result, center, (int)xyr[2], new Scalar(255, 100, 75), 4);
				Imgproc.circle(result, center, 3, new Scalar(255, 100, 75), 5);
			}
		}
		return result;
	}
}
