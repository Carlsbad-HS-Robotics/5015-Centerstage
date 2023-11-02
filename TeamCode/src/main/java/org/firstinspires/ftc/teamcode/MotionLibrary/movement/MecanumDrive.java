package org.firstinspires.ftc.teamcode.MotionLibrary.movement;

import static java.lang.Math.asin;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MotionLibrary.util.*;
import org.firstinspires.ftc.teamcode.MotionLibrary.util.path.Path;

/**
 * Class for controlling movements during opModes
 */
abstract class MecanumDrive {

    public DcMotor leftFront, leftBack, rightBack, rightFront;
    private Ultrasonic frontRightUltrasonic, frontLeftUltrasonic, rightUltrasonic, leftUltrasonic;
    public BNO055IMU imu;
    public double startingAngle;
    public double turnSpeedMultiply = 1;

    public Pose2D currentPosition = new Pose2D();

    //Constants
    //Ultrasonic distances from the top
    public double frontRightUltrasonicDistance = 1;
    public double frontLeftUltrasonicDistance = 1;
    public double rightUltrasonicDistance = 1;
    public double leftUltrasonicDistance = 1;

    //Ultrasonic distance from the side
    public double frontRightUltrasonicAnglePitch = 0;
    public double frontLeftUltrasonicAnglePitch = 0;
    public double rightUltrasonicAngleRoll = 0;
    public double leftUltrasonicAngleRoll = 0;
    public double frontRightUltrasonicDistancePitch = 1;
    public double frontLeftUltrasonicDistancePitch = 1;
    public double rightUltrasonicDistanceRoll = 1;
    public double leftUltrasonicDistanceRoll = 1;


    public MecanumDrive(OpMode opMode) {
        leftFront = opMode.hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = opMode.hardwareMap.get(DcMotor.class, "leftRear");
        rightBack = opMode.hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = opMode.hardwareMap.get(DcMotor.class, "rightFront");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
        //Ultrasonic
        frontRightUltrasonic = new Ultrasonic(opMode.hardwareMap.get(AnalogInput.class, "fr_ultrasonic"));
        frontLeftUltrasonic = new Ultrasonic(opMode.hardwareMap.get(AnalogInput.class, "fl_ultrasonic"));
        rightUltrasonic = new Ultrasonic(opMode.hardwareMap.get(AnalogInput.class, "br_ultrasonic"));
        leftUltrasonic = new Ultrasonic(opMode.hardwareMap.get(AnalogInput.class, "bl_ultrasonic"));
         */

        //IMU init
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");

        PID.setConstantsMove(1,1,1);
    }

    /**
     * Method to track the position of the during movement.
     * Position is stored in currentPosition Pose2d.
     * Keep looping through this method to keep the current position accurate
     */
    public void trackPosition() {

    }

    // ToDo: Switch imu axes to align with orientation of robot

    /*
                            /\
                             |   x-axis / Pitch
                             |
                             |
     ______________________________________________
    |                                             |
    |                                             |
    |                                             |
    |                                             |
    |                Control Hub                  |  /_____ y-axis / Roll
    |                                             |  \
    |                                             |
    |                                             |
    |_____________________________________________|

     */

    public double ultrasonicFront;
    public double ultrasonicSide;
    public double distanceToPoint;
    public double pitchRollAngle;
    public double yawAngle;
    public double distanceToPointAngleOffset;
    Vector2D UltrasonicPosition = new Vector2D();
    public int ultrasonicSideMultiplier;

    public Vector2D UltrasonicPositionRight() {
        trackPosition();

        yawAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        if (currentPosition.heading >= 0){
            ultrasonicFront = frontLeftUltrasonic.getDistance();

            //Pitch correction
            pitchRollAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            distanceToPoint = sqrt(pow(frontLeftUltrasonicDistancePitch, 2 ) + pow(ultrasonicFront, 2)
                    - 2 * frontLeftUltrasonicDistancePitch * ultrasonicFront * cos(frontLeftUltrasonicAnglePitch));
            ultrasonicFront = distanceToPoint * cos(pitchRollAngle - asin(ultrasonicFront * sin(frontLeftUltrasonicAnglePitch) / distanceToPoint));

            //Angle Correction

            distanceToPoint = sqrt( pow(frontLeftUltrasonicDistance, 2) + pow( ultrasonicFront, 2) );
            UltrasonicPosition.Y = distanceToPoint * ( yawAngle - atan(ultrasonicFront / frontLeftUltrasonicDistance) );

        } else {
            ultrasonicFront = frontRightUltrasonic.getDistance();

            //Pitch correction
            pitchRollAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            distanceToPoint = sqrt(pow(frontRightUltrasonicDistancePitch, 2 ) + pow(ultrasonicFront, 2)
                    - 2 * frontRightUltrasonicDistancePitch * ultrasonicFront * cos(frontRightUltrasonicAnglePitch));
            ultrasonicFront = distanceToPoint * cos(pitchRollAngle - asin(ultrasonicFront * sin(frontRightUltrasonicAnglePitch) / distanceToPoint));

            //Angle Correction

            distanceToPoint = sqrt( pow(frontRightUltrasonicDistance, 2) + pow( ultrasonicFront, 2) );
            UltrasonicPosition.Y = distanceToPoint * ( yawAngle - atan(ultrasonicFront / frontRightUltrasonicDistance) );
        }
        ultrasonicSide = rightUltrasonic.getDistance();

        //Roll correction
        pitchRollAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        distanceToPoint = sqrt(pow(rightUltrasonicDistanceRoll, 2 ) + pow(ultrasonicSide, 2)
                - 2 * rightUltrasonicDistanceRoll * ultrasonicFront * cos(rightUltrasonicAngleRoll));
        ultrasonicFront = distanceToPoint * cos(pitchRollAngle - asin(ultrasonicSide * sin(rightUltrasonicAngleRoll) / distanceToPoint));

        //Angle Correction

        distanceToPoint = sqrt( pow(rightUltrasonicDistance, 2) + pow( ultrasonicSide, 2) );
        UltrasonicPosition.X = distanceToPoint * ( yawAngle - atan(ultrasonicSide / rightUltrasonicDistance) );

        return UltrasonicPosition;
    }

    public Vector2D trackPositionUltrasonicLeft() {
        trackPosition();

        ultrasonicSideMultiplier = -1;

        yawAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        if (currentPosition.heading >= 0){
            ultrasonicFront = frontLeftUltrasonic.getDistance();
            //Pitch correction
            pitchRollAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            distanceToPoint = sqrt(pow(frontLeftUltrasonicDistancePitch, 2 ) + pow(ultrasonicFront, 2)
                    - 2 * frontLeftUltrasonicDistancePitch * ultrasonicFront * cos(frontLeftUltrasonicAnglePitch));
            ultrasonicFront = distanceToPoint * cos(pitchRollAngle - asin(ultrasonicFront * sin(frontLeftUltrasonicAnglePitch) / distanceToPoint));

            //Angle Correction

            distanceToPoint = sqrt( pow(frontLeftUltrasonicDistance, 2) + pow( ultrasonicFront, 2) );
            UltrasonicPosition.Y = distanceToPoint * ( yawAngle - atan(ultrasonicFront / frontLeftUltrasonicDistance) );

        } else {
            ultrasonicFront = frontRightUltrasonic.getDistance();

            //Pitch correction
            pitchRollAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            distanceToPoint = sqrt(pow(frontRightUltrasonicDistancePitch, 2 ) + pow(ultrasonicFront, 2)
                    - 2 * frontRightUltrasonicDistancePitch * ultrasonicFront * cos(frontRightUltrasonicAnglePitch));
            ultrasonicFront = distanceToPoint * cos(pitchRollAngle - asin(ultrasonicFront * sin(frontRightUltrasonicAnglePitch) / distanceToPoint));

            //Angle Correction

            distanceToPoint = sqrt( pow(frontRightUltrasonicDistance, 2) + pow( ultrasonicFront, 2) );
            UltrasonicPosition.Y = distanceToPoint * ( yawAngle - atan(ultrasonicFront / frontRightUltrasonicDistance) );
        }
        ultrasonicSide = leftUltrasonic.getDistance();

        //Roll correction
        pitchRollAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        distanceToPoint = sqrt(pow(leftUltrasonicDistanceRoll, 2 ) + pow(ultrasonicSide, 2)
                - 2 * leftUltrasonicDistanceRoll * ultrasonicFront * cos(leftUltrasonicAngleRoll));
        ultrasonicFront = distanceToPoint * cos(pitchRollAngle - asin(ultrasonicSide * sin(leftUltrasonicAngleRoll) / distanceToPoint));

        //Angle Correction

        distanceToPoint = sqrt( pow(leftUltrasonicDistance, 2) + pow( ultrasonicSide, 2) );
        UltrasonicPosition.X = distanceToPoint * ( yawAngle - atan(ultrasonicSide / leftUltrasonicDistance) );

        return UltrasonicPosition;
    }

    Vector2D ultrasonicPositionSet = new Vector2D();
    public Vector2D rotateUltrasonic(Vector2D ultrasonicPosition) {
        trackPosition();
        if (currentPosition.heading >= 0) {
            if (currentPosition.heading <= 45) {
                ultrasonicPositionSet.Y = ultrasonicPosition.Y;
                ultrasonicPositionSet.X = ultrasonicPosition.X * ultrasonicSideMultiplier;
            } else if (currentPosition.heading <= 135) {
                ultrasonicPositionSet.Y = ultrasonicPosition.X * ultrasonicSideMultiplier;
                ultrasonicPositionSet.X = ultrasonicPosition.Y;
            } else {
                ultrasonicPositionSet.Y = -ultrasonicPosition.Y;
                ultrasonicPositionSet.X = -ultrasonicPosition.X * ultrasonicSideMultiplier;
            }
        } else {
            if (currentPosition.heading >= 45) {
                ultrasonicPositionSet.Y = ultrasonicPosition.Y;
                ultrasonicPositionSet.X = ultrasonicPosition.X * ultrasonicSideMultiplier;
            } else if (currentPosition.heading >= 135) {
                ultrasonicPositionSet.Y = ultrasonicPosition.X * ultrasonicSideMultiplier;
                ultrasonicPositionSet.X = -ultrasonicPosition.Y;
            } else {
                ultrasonicPositionSet.Y = -ultrasonicPosition.Y;
                ultrasonicPositionSet.X = -ultrasonicPosition.X * ultrasonicSideMultiplier;
            }
        }

        ultrasonicSideMultiplier = 1;
        return ultrasonicPositionSet;
    }

    //Movement stuff

    /**
     * Sets the currentPosition to a specific position
     */
    public void setPositionTo(Vector2D positionSet) {

    }

    /**
     * Sets the currentPosition to a specific position
     */
    public void setPositionTo(Pose2D positionSet) {

    }

    public void turn(double heading){
        heading *= turnSpeedMultiply;
        leftFront.setPower(Range.clip( heading, -1, 1 ));
        leftBack.setPower(Range.clip( heading, -1, 1 ));
        rightFront.setPower(Range.clip( -heading, -1, 1 ));
        rightBack.setPower(Range.clip( -heading, -1, 1 ));
    }

    /**
     * Move along a Vector2d.
     * Robot centric movement
     */
    public void move(Vector2D move) {
        move.capAt1();
        leftFront.setPower(Range.clip(-move.X + move.Y, -1, 1));
        leftBack.setPower(Range.clip(move.X + move.Y, -1, 1));
        rightBack.setPower(Range.clip(-move.X + move.Y, -1, 1));
        rightFront.setPower(Range.clip(move.X + move.Y, -1, 1));
    }

    /**
     * Move along a Vector2d.
     * Robot centric movement
     */
    public void move(Vector2D move, double speed) {
        move.capAt1();
        leftFront.setPower(Range.clip((-move.X + move.Y) * speed, -1, 1));
        leftBack.setPower(Range.clip((move.X + move.Y) * speed, -1, 1));
        rightBack.setPower(Range.clip((-move.X + move.Y) * speed, -1, 1));
        rightFront.setPower(Range.clip((move.X + move.Y) * speed, -1, 1));
    }

    /**
     * Move along a Vector2d.
     * Robot centric movement
     */
    public void move(Pose2D move) {
        move.capAt1();
        move.heading *= turnSpeedMultiply;
        leftFront.setPower(Range.clip(-move.X + move.Y + move.heading, -1, 1));
        leftBack.setPower(Range.clip(move.X + move.Y + move.heading, -1, 1));
        rightBack.setPower(Range.clip(-move.X + move.Y - move.heading, -1, 1));
        rightFront.setPower(Range.clip(move.X + move.Y - move.heading, -1, 1));
    }

    /**
     * Move along a Vector2d.
     * Robot centric movement
     */
    public void move(Pose2D move, double speed) {
        move.capAt1();
        move.heading *= turnSpeedMultiply;
        leftFront.setPower(Range.clip(-move.X + move.Y + move.heading * speed, -1, 1));
        leftBack.setPower(Range.clip(move.X + move.Y + move.heading * speed, -1, 1));
        rightBack.setPower(Range.clip(-move.X + move.Y - move.heading * speed, -1, 1));
        rightFront.setPower(Range.clip(move.X + move.Y - move.heading * speed, -1, 1));
    }

    /**
     * Moves forward and reverse at a given speed
     */
    public void moveForward(double speed) {
        leftFront.setPower((Range.clip( speed, -1, 1 )));
        leftBack.setPower((Range.clip( speed, -1, 1 )));
        rightBack.setPower((Range.clip( speed, -1, 1 )));
        rightFront.setPower((Range.clip( speed, -1, 1 )));
    }

    /**
     * Sets all motor powers to 0
     */
    public void stopMotors() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    /**
     * Moves towards a point on the field
     * Field centric movement
     */
    public void moveTowards(Vector2D target) {
        trackPosition();
        Vector2D error = Vector2D.sub( new Vector2D(currentPosition), target);
        move(error);
    }

    /**
     * Moves towards a point on the field
     * Field centric movement
     */
    public void moveTowards(Vector2D target, double speed) {
        trackPosition();
        Vector2D error = Vector2D.sub( new Vector2D(currentPosition), target);
        move(error, speed);
    }

    /**
     * Moves towards a point on the field
     * Field centric movement
     */
    public void moveTowards(Pose2D target, double speed) {
        trackPosition();
        double headingError;
        if (Math.abs(currentPosition.heading - target.heading) >= Math.abs(currentPosition.heading - target.heading - 360)){
            headingError = currentPosition.heading - target.heading - 360;
        } else {
            headingError = currentPosition.heading - target.heading;
        }
        Pose2D error = Pose2D.sub(currentPosition, target);
        move( new Pose2D( new Vector2D(error), headingError), speed);
    }

    /**
     * Turns towards a specified field centric heading
     */
    public void turnTo(double heading) {
        PID turnPID = new PID(PID.Type.turn);
        double headingError = 1;
        while (headingError != 0){
            trackPosition();
            if (Math.abs(currentPosition.heading - heading) >= Math.abs(currentPosition.heading - heading - 360)){
                headingError = currentPosition.heading - heading - 360;
            } else {
                headingError = currentPosition.heading - heading;
            }
            turnPID.pid(headingError);
            turn(turnPID.pidout);
        }
        stopMotors();
    }

    /**
     * Moves linearly to a specified point on the field
     */
    public void lineTo(Vector2D target) {
        NanoClock Time = new NanoClock();
        Vector2D error = new Vector2D(1);
        PID pidX = new PID(PID.Type.move);
        PID pidY = new PID(PID.Type.move);
        while (error.X != 0 && error.Y != 0) {
            trackPosition();
            error = Vector2D.sub( Pose2D.Vector2d(currentPosition), target);
            pidX.pid(error.X);
            pidY.pid(error.Y);
            move(new Vector2D(pidX.pidout, pidY.pidout));
        }
        stopMotors();
    }

    /**
     * Moves linearly to a specified point on the field while changing heading to a specified heading
     */
    public void lineToSplineHeading(Pose2D target) {
        NanoClock Time = new NanoClock();
        Pose2D error = new Pose2D(1);
        PID pidX = new PID(PID.Type.move);
        PID pidY = new PID(PID.Type.move);
        PID pidHeading = new PID(PID.Type.turn);
        double headingError = 0;
        while (error.X != 0 & error.Y != 0 & headingError != 0) {
            trackPosition();
            error = Pose2D.sub(currentPosition, target);
            pidX.pid(error.X);
            pidY.pid(error.Y);
            pidHeading.pid(error.heading);

            //Heading
            if (Math.abs(currentPosition.heading - target.heading) >= Math.abs(currentPosition.heading - target.heading - 360)) {
                headingError = currentPosition.heading - target.heading - 360;
            } else {
                headingError = Math.abs(currentPosition.heading - target.heading);
            }
            pidHeading.pid(headingError);
            move(new Pose2D(pidX.pidout, pidY.pidout, pidHeading.pidout));

        }
        stopMotors();
    }

    /**
     * Moves along a Path
     * @param segmentSteps Number of points in each segment of the Path that the robot is following
     */
    public void followPath(Path path, int segmentSteps) {
        PID error = new PID(PID.Type.move);
        Pose2D Error = new Pose2D(1);
        path.initSegmentPos(currentPosition, segmentSteps);
        while (Error.getMagnitude() != 0 && Error.heading != 0) {
            Error = Pose2D.sub(currentPosition, path.end);
            error.pid(Pose2D.sub(currentPosition, path.end).getMagnitude());
            moveTowards(path.posePath.getClosestPoint(currentPosition)  , error.pidout);
        }
        stopMotors();
    }

    /**
     * Moves along a Path starting at the last poing and going backwards
     * @param segmentSteps Number of points in each segment of the Path that the robot is following
     */
    public void followPathBack(Path path, int segmentSteps) {
        PID error = new PID(PID.Type.move);
        Pose2D Error = new Pose2D(1);
        path.initSegmentPos(currentPosition, -segmentSteps);
        while (Error.getMagnitude() != 0 && Error.heading != 0) {
            Error = Pose2D.sub(currentPosition, path.end);
            error.pid(Pose2D.sub(currentPosition, path.end).getMagnitude());
            moveTowards(path.posePath.getClosestPointBack(currentPosition)  , error.pidout);
        }
        stopMotors();
    }
}