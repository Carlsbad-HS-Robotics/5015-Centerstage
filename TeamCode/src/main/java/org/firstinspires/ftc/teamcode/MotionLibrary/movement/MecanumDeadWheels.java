package org.firstinspires.ftc.teamcode.MotionLibrary.movement;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MotionLibrary.util.*;




public class MecanumDeadWheels extends MecanumDrive {

    //Constants
    public double distanceBetweenEncoders = 20; //inches
    public double frontOffset = 0; //Distance of dead wheels from center of rotation (positive is forward, negative is back)
    public double encoderTicsPerInch = 8192 / 1.37795 * PI;

    private Encoder frontEncoder, rightEncoder, leftEncoder;


    public MecanumDeadWheels(@NonNull OpMode opMode, double startingangle) {
        super(opMode);

        //Encoder init
        //IMPORTANT: Change the device name to the name of the motor that the encoder is plugged into
        frontEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "frontEncoder"), 1, encoderTicsPerInch);
        rightEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "rightEncoder"), 1, encoderTicsPerInch);
        leftEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "leftEncoder"), 1, encoderTicsPerInch);

        //IMU init
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");

        startingAngle = startingangle;
    }

    //Dead Wheels Stuff

    Pose2D currentBotCentricPosition = new Pose2D();
    Pose2D previousBotCentricPosition = new Pose2D();
    Pose2D botCentricDifference = new Pose2D();
    public double currentAngle;

    @Override
    public void trackPosition() {
        currentAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle + startingAngle;
        currentBotCentricPosition = new Pose2D(frontEncoder.getCurrentPosition(), (rightEncoder.getCurrentPosition() + leftEncoder.getCurrentPosition()) / 2);
        botCentricDifference = Pose2D.sub(currentBotCentricPosition, previousBotCentricPosition);
        botCentricDifference.rotateRadians(Math.toRadians(-currentAngle));
        botCentricDifference.div(encoderTicsPerInch);
        currentPosition.add(botCentricDifference);
        currentPosition.heading = currentAngle;
        previousBotCentricPosition = currentBotCentricPosition;
    }

    //Movement stuff

    @Override
    public void setPositionTo(Vector2D positionSet) {
        trackPosition();
        currentPosition = new Pose2D(positionSet);
    }

    @Override
    public void setPositionTo(Pose2D positionSet) {
        trackPosition();
        currentPosition = positionSet;
    }


}
