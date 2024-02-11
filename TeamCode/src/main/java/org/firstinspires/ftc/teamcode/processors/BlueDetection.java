package org.firstinspires.ftc.teamcode.processors;

import android.graphics.ColorSpace;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlueDetection extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */

    public enum ObjectPosition {
        LEFT,
        CENTER,
        RIGHT,
        NOT_FOUND
    }

    // TOPLEFT anchor point for the bounding box
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(19,188);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(210,188);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(520,188);
    static final int REGION_WIDTH = 20;
    static final int REGION_HEIGHT = 20;

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_blue_bounds = new Scalar(110,50,50),
            upper_red_bounds = new Scalar(130,255,255);

    // Color definitions
    private final Scalar
            BLUE = new Scalar(240, 100, 100),
            WHITE = new Scalar(255,255,255);


    // Percent and mat definitions
    private double leftPercent, rightPercent, midPercent;
    private Mat leftMat = new Mat(),
            rightMat = new Mat(),
            midMat = new Mat(),
            blurredMat = new Mat(),
            blurredMat1 = new Mat(),
            blurredMat2 = new Mat(),
            blurredMat3 = new Mat(),
            kernel = new Mat();

    // Anchor point definitions
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH+30,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH+30,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    // Running variable storing the parking position
    public volatile ObjectPosition position;
    private Telemetry telemetry;
    public void telemetry_added(){

        /*
        telemetry.addData("[Pattern]", position);
        telemetry.addData("[leftPercent]", leftPercent);
        telemetry.addData("[rightPercent]", rightPercent);
        telemetry.addData("[midPercent]", midPercent);
        telemetry.update();
         */
    }
    public BlueDetection(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2HSV );
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        blurredMat1 = blurredMat.submat(new Rect(region1_pointA, region1_pointB));
        blurredMat2 = blurredMat.submat(new Rect(region2_pointA, region2_pointB));
        blurredMat3 = blurredMat.submat(new Rect(region3_pointA, region3_pointB));

        // Apply Morphology
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat1, lower_blue_bounds, upper_red_bounds, leftMat);
        Core.inRange(blurredMat2, lower_blue_bounds, upper_red_bounds, midMat);
        Core.inRange(blurredMat3, lower_blue_bounds, upper_red_bounds, rightMat);

        // Gets color specific values
        leftPercent = Core.countNonZero(leftMat);
        rightPercent = Core.countNonZero(rightMat);
        midPercent = Core.countNonZero(midMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(midPercent, Math.max(rightPercent, leftPercent));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent == 0) {
            position = ObjectPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    region3_pointA,
                    region3_pointB,
                    WHITE,
                    2
            );
        }
        else if (maxPercent == leftPercent) {
            position = ObjectPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    region3_pointA,
                    region3_pointB,
                    WHITE,
                    2
            );
        } else if (maxPercent == midPercent) {
            position = ObjectPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    BLUE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    region3_pointA,
                    region3_pointB,
                    WHITE,
                    2
            );        } else if (maxPercent == rightPercent) {
            position = ObjectPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    region3_pointA,
                    region3_pointB,
                    BLUE,
                    2
            );        }

        // Memory cleanup
        blurredMat.release();
        leftMat.release();
        rightMat.release();
        midMat.release();
        blurredMat1.release();
        blurredMat2.release();
        blurredMat3.release();
        kernel.release();
        telemetry_added();
        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ObjectPosition getPosition() {
        return position;
    }

}