//package org.firstinspires.ftc.teamcode.opencv;
//
//import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class SpecRotationPipeline extends OpenCvPipeline {
//    Mat image;
//
//    public void init(int width, int height, CameraCalibration calibration) {
//
//    }
//
//    @Override
//    public Object processFrame(Mat frame) {
//        Mat specimenMask = specimenThreshold(frame);
//
//        List<MatOfPoint> contourList = new ArrayList<>();
//        Mat heirarchy = new Mat();
//
//        Imgproc.findContours(specimenMask, contourList, heirarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
//
//        specimenContour = findSpecimenContour(contourList);
//
//
//    }
//
//    public MatOfPoint findSpecimenContour(List<MatOfPoint> contours) {
//        MatOfPoint largestContour = contours.get(0);
//        double maxArea = Imgproc.contourArea(largestContour);
//
//        for (MatOfPoint contour : contours) {
//            double currentArea = Imgproc.contourArea(contour);
//
//            if (currentArea > maxArea) {
//                largestContour = contour;
//                maxArea = currentArea;
//            }
//        }
//
//        return largestContour;
//
//    }
//
//    public Mat specimenThreshold(Mat frame) {
//        Mat threshold;
//
//        int height = frame.rows();
//        int width = frame.cols();
//
//        int midHeight = (int) (height / 2);
//        int midWidth = (int) (width / 2);
//
//        double[] centerPixel = frame.get(midHeight, midWidth);
//
//        double[] lowerHSV;
//        double[] higherHSV;
//        // Checks the color of the central pixel by comparing the BGR values
//        // if B >> G and R, then it is blue
//        if (centerPixel[0] > centerPixel[1] + 25 && centerPixel[0] > centerPixel[2] + 25) {
//            lowerHSV = [];
//            higherHSV = [];
//        }
//
//        else-if (centerPixel[1] > centerPixel[0] + 25 && centerPixel[1] > centerPixel[2] + 25) {
//            lowerHSV = [];
//            higherHSV = [];
//        }
//
//        else-if (centerPixel[2] > centerPixel[0] + 25 && centerPixel[2] > centerPixel[1] + 25) {
//            lowerHSV = [];
//            higherHSV = [];
//        }
//
//
//        threshold = Imgproc.inRange(frame, lowerHSV, higherHSV);
//
//
//    }
//
//
//}
