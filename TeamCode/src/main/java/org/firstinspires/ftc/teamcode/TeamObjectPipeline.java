package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamObjectPipeline extends OpenCvPipeline {
    private double leftPerc,
    midPerc,
    rightPerc;
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT1 = new Point(480, 440);
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT2 = new Point(680, 440);
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT3 = new Point(880, 440);
    private static final Scalar
            lower_yellow_bounds  = new Scalar(45, 255, 255),
            upper_yellow_bounds  = new Scalar(75, 255, 255);
    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;
    Point rect_pointA1 = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT1.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT1.y);
    Point rect_pointB1 = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT1.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT1.y + REGION_HEIGHT);

    Point rect_pointA2 = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT2.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT2.y);
    Point rect_pointB2 = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT2.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT2.y + REGION_HEIGHT);
    Point rect_pointA3 = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT3.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT3.y);
    Point rect_pointB3 = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT3.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT3.y + REGION_HEIGHT);



    @Override
    public Mat processFrame(Mat input) {
        leftPerc = getPerc();
        return input;
    }
    private double getPerc(Mat gat) {
        double perc;
        Mat blurredMat = new Mat(),
                kernel = new Mat();
        Imgproc.blur(gat, blurredMat, new Size(5, 5));
        blurredMat = blurredMat.submat(new Rect(rect_pointA1, rect_pointB1));
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);
        Core.inRange(blurredMat,lower_yellow_bounds,upper_yellow_bounds,blurredMat);
        perc = Core.countNonZero(blurredMat);
        return perc;
    }
}
