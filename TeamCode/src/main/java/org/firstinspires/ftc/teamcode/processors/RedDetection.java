package org.firstinspires.ftc.teamcode.processors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedDetection extends OpenCvPipeline {
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
            lower_red_bounds1 = new Scalar(0,70,50),
            upper_red_bounds1 = new Scalar(10,255,255),
            lower_red_bounds2 = new Scalar(170,70,50),
            upper_red_bounds2 = new Scalar(180,255,255);

    // Color definitions
    private final Scalar
            RED = new Scalar(255, 0, 0),
    WHITE = new Scalar(255,255,255);


    // Percent and mat definitions
    private double leftPercent, rightPercent, midPercent;
    private Mat leftMat1 = new Mat(),
            rightMat1= new Mat(),
            midMat1 = new Mat(),
            leftMat = new Mat(),
            rightMat= new Mat(),
            midMat = new Mat(),
            leftMat2 = new Mat(),
            rightMat2= new Mat(),
            midMat2 = new Mat(),
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
    public volatile ObjectPosition position = ObjectPosition.CENTER ;
    private Telemetry telemetry;
    public void telemetry_added(){

        telemetry.addData("[Pattern]", position);
        telemetry.addData("[yelPercent]", leftPercent);
        telemetry.addData("[cyaPercent]", rightPercent);
        telemetry.addData("[magPercent]", midPercent);
        telemetry.update();
    }
    public RedDetection(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2HSV);
        // Noise reduction
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        blurredMat1 = blurredMat.submat(new Rect(region1_pointA, region1_pointB));
        blurredMat2 = blurredMat.submat(new Rect(region2_pointA, region2_pointB));
        blurredMat3 = blurredMat.submat(new Rect(region3_pointA, region3_pointB));

        // Apply Morphology
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat1, lower_red_bounds1, upper_red_bounds1, leftMat1);
        Core.inRange(blurredMat2, lower_red_bounds1, upper_red_bounds1, midMat1);
        Core.inRange(blurredMat3, lower_red_bounds1, upper_red_bounds1, rightMat1);
        Core.inRange(blurredMat1, lower_red_bounds2, upper_red_bounds2, leftMat2);
        Core.inRange(blurredMat2, lower_red_bounds2, upper_red_bounds2, midMat2);
        Core.inRange(blurredMat3, lower_red_bounds2, upper_red_bounds2, rightMat2);

        // Gets color specific values
        Core.bitwise_or(leftMat1,leftMat2,leftMat);
        Core.bitwise_or(midMat1,midMat2,midMat);
        Core.bitwise_or(rightMat1,rightMat2,rightMat);
        leftPercent = Core.countNonZero(leftMat);
        rightPercent = Core.countNonZero(rightMat);
        midPercent = Core.countNonZero(midMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(midPercent, Math.max(rightPercent, leftPercent));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent <= 200) {
            position = ObjectPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    RED,
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
                    RED,
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
                    RED,
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
                    RED,
                    2
            );        }

        // Memory cleanup
        blurredMat.release();
        blurredMat1.release();
        blurredMat2.release();
        blurredMat3.release();
        leftMat.release();
        rightMat.release();
        midMat.release();
        kernel.release();
        telemetry_added();
        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ObjectPosition getPosition() {
        return position;
    }

}