package org.firstinspires.ftc.teamcode.robo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robo.Arm;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.processors.BlueDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "bdAlign")
public class BackdropAlign extends LinearOpMode {
    private AprilTagDetection aprilTagDetection;
    private final int DESIRED_TAG_ID = 1;
    YawPitchRollAngles robotOrientation;
    private AprilTagProcessor aprilTag;
    SampleMecanumDrive drive;
    Pose2d position;
    IMU imu;

    VisionPortal visionPortal;
    double heading;
    double range;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        robotOrientation = imu.getRobotYawPitchRollAngles();
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR,
                DriveConstants.USB_FACING_DIR
        ));

        imu.initialize(parameters);
        aprilTag        = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal            .easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        heading        = robotOrientation.getPitch(AngleUnit.DEGREES);
        drive             = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        if(isStopRequested()) return;

        while(opModeIsActive()) {
            ArrayList<AprilTagDetection>  arr = aprilTag.getDetections();
            if(heading>92){
                turnLeft();
            }
            if(heading<88){
                turnRight();
            }
            if(heading<92&&heading>88){
                stopMoving();
            }

            if(!(arr == null)){
                for (AprilTagDetection detection : arr) {
                    if (detection.id == DESIRED_TAG_ID) {
                        range             = detection.ftcPose.range;
                    }
                }
            }

            position = new Pose2d (heading, range);

            telemetry.addLine("Centering Backdrop");
            telemetry.addLine("==============");
            telemetry.addData("Heading", heading);
            telemetry.addData("Distance", range);
            telemetry.update();
        }
    }

    /*
    private void initAprilTag(){
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1136.7,1136.7, 281.597,138.901)// constants from calibrating camera with 3d zephyr
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
     */

    public void turnRight() {
        drive.leftFront.setPower(0.1);
        drive.leftRear.setPower(0.1);
        drive.rightFront.setPower(-0.1);
        drive.rightRear.setPower(-0.1);
    }

    public void turnLeft() {
        drive.leftFront.setPower(-0.1);
        drive.leftRear.setPower(-0.1);
        drive.rightFront.setPower(0.1);
        drive.rightRear.setPower(0.1);
    }

    public void stopMoving() {
        drive.leftFront.setPower(0);
        drive.leftRear.setPower(0);
        drive.rightFront.setPower(0);
        drive.rightRear.setPower(0);
    }



}

