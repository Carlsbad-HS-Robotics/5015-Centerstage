package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Autonomous(name="AutoSafe")
public class SafeAuto extends LinearOpMode {

    SampleMecanumDrive drive;
    enum State{
        TRAJECTORY_1,
        TURN_RIGHT,
        TURN_LEFT,
        DROP1,
        TRAJECTORY2,
        TURN1,
        TRACK,
        DROP2,
        IDLE
    }
    Arm arm_subsystem;
    OpenCvCamera webcam;
    volatile BlueDetection.ObjectPosition position;
    private AprilTagDetection aprilTagDetection;
    private AprilTagProcessor aprilTag;
    private final int DESIRED_TAG_ID = 1;
    double poseBearing;
    VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        waitForStart();
        //position = pipeline.getPosition();

        if(isStopRequested()) return;
        while(opModeIsActive()) {
            ArrayList<AprilTagDetection>  arr = aprilTag.getDetections();
            if(!(arr == null)){
                for (AprilTagDetection detection : arr) {
                    if (detection.id == DESIRED_TAG_ID) {
                        poseBearing = detection.ftcPose.bearing;
                    }
                    if(poseBearing < -2){
                        turnRight();
                    }
                    if(poseBearing > 2){
                        turnLeft();
                    }
                    if(poseBearing > -2 && poseBearing < 2){
                        stopMoving();
                    }
                }
            }

       //     telemetry.addData("imu", Math.toDegrees(drive.getExternalHeading()));
            telemetry.addData("pos", position);
            telemetry.addData("pose Bearing", poseBearing);
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
