package org.firstinspires.ftc.teamcode.opencv;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class RedPropPipeline implements VisionProcessor {
    Mat hsvFrame;

    Scalar lowerBlue;
    Scalar upperBlue;

    Mat highMat = new Mat();
    Mat lowMat = new Mat();

    public Scalar lowHSVRedLower = new Scalar(0, 141.7, 59.5);  //Beginning of Color Wheel
    public Scalar lowHSVRedUpper = new Scalar(0.6, 255, 255);

    public Scalar redHSVRedLower = new Scalar(175.3, 100, 100); //Wraps around Color Wheel
    public Scalar highHSVRedUpper = new Scalar(179, 255, 202.6);

    Rect leftRect = new Rect(
            new Point(0, 0),
            new Point(150, 448)
    );

    double width = 0;
    public double centerX = 0;
    double centerY = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat blueMask = preProcessFrame(frame);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(blueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint largestContour = findLargestContour(contours);

        if (largestContour != null) {
            Imgproc.drawContours(frame, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);

            width = calculateWidth(largestContour);

            String widthLabel = "Width: " + (int) width + " px";
            Imgproc.putText(frame, widthLabel, new org.opencv.core.Point(10, 50), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);

            //Calculate center
            Rect boundingRect = Imgproc.boundingRect(largestContour);
            centerX = boundingRect.x + (boundingRect.width / 2);
            centerY = boundingRect.y + (boundingRect.height / 2);

            String centerLabel = "Center: (" + (int) centerX + ", " + (int) centerY + ")";
            Imgproc.putText(frame, centerLabel, new Point(10, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);
            Imgproc.circle(frame, new org.opencv.core.Point(centerX, centerY), 5, new Scalar(0, 255, 0), 2);

            double leftBox = Core.sumElems(blueMask.submat(leftRect)).val[0];
            double averagedLeftBox = leftBox / leftRect.area() / 255;

            if (averagedLeftBox > 0.5) {
                Imgproc.putText(frame, "Prop Left", new Point(10, 150), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);
            }

        }
        return null;
    }

    public Mat preProcessFrame(Mat frame) {
        hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        Mat blueMask = new Mat();

        Core.inRange(hsvFrame, lowHSVRedLower, lowHSVRedUpper, lowMat);
        Core.inRange(hsvFrame, redHSVRedLower, highHSVRedUpper, highMat);

        hsvFrame.release();

        Core.bitwise_or(lowMat, highMat, blueMask);

        lowMat.release();
        highMat.release();

        Mat kernel = new Mat();
        Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);


        Imgproc.erode(blueMask, blueMask, kernel);
        Imgproc.dilate(blueMask, blueMask, kernel);

        return blueMask;
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public boolean isPropCenter() {
        return centerX > 0 && centerX < 560;
    }
    public boolean isPropRight() {
        return centerX > 580 && centerX < 800;
    }

}
