package org.firstinspires.ftc.teamcode.MotionLibrary.movement;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MotionLibrary.util.*;

public class MecanumDriveEncoders extends MecanumDrive {

    private Encoder leftFrontEncoder, rightFrontEncoder, leftRearEncoder, rightRearEncoder;

    public double startingAngle;
    public double encoderTicsPerRev = 537.7;
    public double inchesPerRev = PI * 3.7;
    public double encoderTicsPerInch = encoderTicsPerRev * inchesPerRev;



    public MecanumDriveEncoders(@NonNull OpMode opMode, double startingangle) {
         super(opMode);

        leftFrontEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "leftFront"), 1, encoderTicsPerInch);
        rightFrontEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "rightFront"), 1, encoderTicsPerInch);
        leftRearEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "leftRear"), 1, encoderTicsPerInch);
        rightRearEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "rightRear"), 1, encoderTicsPerInch);

        startingAngle = startingangle;
    }

    //Mecanum Wheel Encoder Calculations

    Vector2D currentBotCentricPosition = new Vector2D(0, 0 );
    public double currentAngle;
    Vector2D previousLeftFrontVector = new Vector2D(0, 0);
    Vector2D previousLeftRearVector = new Vector2D(0, 0);
    Vector2D previousRightFrontVector = new Vector2D(0, 0);
    Vector2D previousRightRearVector = new Vector2D(0, 0);
    Vector2D currentLeftFrontVector = new Vector2D(1);
    Vector2D currentRightFrontVector = new Vector2D(-1);
    Vector2D currentLeftRearVector = new Vector2D(1);
    Vector2D currentRightRearVector = new Vector2D(-1);

    double encoderAngleBot;

    @Override
    public void trackPosition() {
        //currentAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle + startingAngle;
        currentLeftFrontVector.scale(leftFront.getCurrentPosition() * encoderTicsPerInch);
        currentRightFrontVector.scale(-rightFront.getCurrentPosition() * encoderTicsPerInch);
        currentLeftRearVector.scale(-leftBack.getCurrentPosition() * encoderTicsPerInch);
        currentRightRearVector.scale(rightBack.getCurrentPosition() * encoderTicsPerInch);
        Vector2D leftFrontDifference = Vector2D.sub(currentLeftFrontVector, previousLeftFrontVector);
        Vector2D leftRearDifference = Vector2D.sub(currentLeftRearVector, previousLeftRearVector);
        Vector2D rightRearDifference = Vector2D.sub(currentRightRearVector, previousRightRearVector);
        Vector2D rightFrontDifference = Vector2D.sub(currentRightFrontVector, previousRightFrontVector);
        previousLeftFrontVector = currentLeftFrontVector;
        previousRightFrontVector = currentRightFrontVector;
        previousLeftRearVector = currentLeftRearVector;
        previousRightRearVector = currentRightRearVector;
        currentBotCentricPosition = Vector2D.add(leftFrontDifference, leftRearDifference, rightFrontDifference, rightRearDifference );
        currentBotCentricPosition.div(4);

        //Current Angle Without IMU
        encoderAngleBot = leftFrontDifference.getMagnitude() + leftRearDifference.getMagnitude() + rightFrontDifference.getMagnitude() * -1 + rightFrontDifference.getMagnitude() * -1;
        currentAngle += 360 * encoderAngleBot/(14 * PI);

        currentBotCentricPosition.rotateRadians(Math.toRadians(-currentAngle));
        currentPosition.add(Vector2D.Pose2d(currentBotCentricPosition));
    }

    //Set Position stuff

    @Override
    public void setPositionTo(Vector2D positionSet) {
        trackPosition();
        currentPosition = new Pose2D(positionSet);
        trackPosition();
    }

    @Override
    public void setPositionTo(Pose2D positionSet) {
        trackPosition();
        currentPosition = positionSet;
    }
}