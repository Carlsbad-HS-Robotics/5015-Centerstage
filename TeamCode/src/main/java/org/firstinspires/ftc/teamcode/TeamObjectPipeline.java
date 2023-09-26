package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamObjectPipeline extends OpenCvPipeline {
    private final Scalar YELLOW  = new Scalar(255, 255, 0), WHITE = new Scalar(255,255,255);
    private double leftPerc,
    midPerc,
    rightPerc,
    maxPerc;
    public enum ObjectPosition {
        LEFT,
        CENTER,
        RIGHT,
    }
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

    public ObjectPosition position = ObjectPosition.CENTER;
    Mat blurredMat = new Mat(),
    kernel = new Mat(),
    leftMat = new Mat(),
    midMat = new Mat(),
    rightMat = new Mat();
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        leftMat = blurredMat.submat(new Rect(rect_pointA1, rect_pointB1));
        midMat = blurredMat.submat(new Rect(rect_pointA2, rect_pointB2));
        rightMat = blurredMat.submat(new Rect(rect_pointA3, rect_pointB3));
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(leftMat, leftMat, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(rightMat, rightMat, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(midMat, midMat, Imgproc.MORPH_CLOSE, kernel);
        Core.inRange(blurredMat,lower_yellow_bounds,upper_yellow_bounds,blurredMat);
        leftPerc = Core.countNonZero(leftMat);
        midPerc = Core.countNonZero(midMat);
        rightPerc = Core.countNonZero(rightMat);
        maxPerc = Math.max(leftPerc,Math.max(rightPerc,midPerc));
        if (maxPerc == leftPerc) {
            position = ObjectPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    rect_pointA1,
                    rect_pointB1,
                    YELLOW,
                    2
            );
            Imgproc.rectangle(
                    input,
                    rect_pointA2,
                    rect_pointB2,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    rect_pointA3,
                    rect_pointB3,
                    WHITE,
                    2
            );

        } else if (maxPerc == midPerc) {
            position = ObjectPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    rect_pointA1,
                    rect_pointB1,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    rect_pointA2,
                    rect_pointB2,
                    YELLOW,
                    2
            );
            Imgproc.rectangle(
                    input,
                    rect_pointA3,
                    rect_pointB3,
                    WHITE,
                    2
            );
        } else if (maxPerc == rightPerc) {
            position = ObjectPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    rect_pointA1,
                    rect_pointB1,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    rect_pointA2,
                    rect_pointB2,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    rect_pointA3,
                    rect_pointB3,
                    YELLOW,
                    2
            );
        }
        leftMat.release();
        rightMat.release();
        kernel.release();
        midMat.release();
        telemetry_added();
        return input;
    }
    public void telemetry_added(){

        telemetry.addData("[Pattern]", position);
        telemetry.addData("[leftPercent]", leftPerc);
        telemetry.addData("[rigPercent]", rightPerc);
        telemetry.addData("[midPercent]", midPerc);
        telemetry.update();
    }
}
