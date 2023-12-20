package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.processors.BlueDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name="AutoFarBlue")
public class FarAutoBlue extends LinearOpMode {

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
    List<AprilTagDetection> myAprilTagDetections;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private int DESIRED_TAG_ID = 1;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        BlueDetection pipeline = new BlueDetection(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        ElapsedTime timer = new ElapsedTime();

        drive = new SampleMecanumDrive(hardwareMap);

        Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                .forward(38)
                .build();
        Trajectory right = drive.trajectoryBuilder(new Pose2d())
                .forward(33)
                .build();
        Trajectory left = drive.trajectoryBuilder(new Pose2d())
                .forward(38)
                .build();

        arm_subsystem = new Arm(hardwareMap);
        State currentState = State.TRAJECTORY_1;


        arm_subsystem.grab();

        sleep(3000);

        telemetry.addData("pos", pipeline.getPosition());
        telemetry.update();



        //initAprilTag();
        waitForStart();

        position = pipeline.getPosition();

        if(isStopRequested()) return;
        if(opModeIsActive()) {
            //myAprilTagDetections = aprilTag.getDetections();
            arm_subsystem.grab();

            telemetry.addData("drive", drive.isBusy());
            telemetry.addData("state", currentState);
            telemetry.addData("imu", Math.toDegrees(drive.getExternalHeading()));
            telemetry.addData("pos", position);
            telemetry.update();

            switch(position){
                case LEFT:
                    arm_subsystem.grab();

                    sleep(500);

                    drive.followTrajectory(left);

                    turnLeft();

                    sleep(1000);

                    stopMoving();

                    stopMoving();

                    arm_subsystem.low();
                    arm_subsystem.update();

                    sleep(1000);

                    arm_subsystem.release();
                    arm_subsystem.update();

                    sleep(2000);

                    arm_subsystem.high();
                    arm_subsystem.update();

                    sleep(1000);

                    arm_subsystem.grab();
                    break;
                case RIGHT:
                    arm_subsystem.grab();

                    sleep(500);

                    drive.followTrajectory(right);

                    turnRight();

                    sleep(500);

                    stopMoving();

                    arm_subsystem.low();
                    arm_subsystem.update();

                    sleep(1000);

                    arm_subsystem.release();
                    arm_subsystem.update();

                    sleep(2000);

                    arm_subsystem.high();
                    arm_subsystem.update();

                    sleep(1000);

                    arm_subsystem.grab();
                    break;
                case CENTER:
                    arm_subsystem.grab();

                    sleep(500);

                    drive.followTrajectory(forward);

                    arm_subsystem.low();
                    arm_subsystem.update();

                    sleep(1000);

                    arm_subsystem.release();
                    arm_subsystem.update();

                    sleep(2000);

                    arm_subsystem.high();
                    arm_subsystem.update();

                    sleep(1000);

                    arm_subsystem.grab();
                    break;
            }
            telemetry.addData("drive", drive.isBusy());
            telemetry.addData("state", currentState);
            telemetry.addData("imu", Math.toDegrees(drive.getExternalHeading()));
            telemetry.addData("pos", position);
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
        drive.leftFront.setPower(0.5);
        drive.leftRear.setPower(0.5);
        drive.rightFront.setPower(-0.5);
        drive.rightRear.setPower(-0.5);
    }

    public void turnLeft() {
        drive.leftFront.setPower(-0.5);
        drive.leftRear.setPower(-0.5);
        drive.rightFront.setPower(0.5);
        drive.rightRear.setPower(0.5);
    }

    public void stopMoving() {
        drive.leftFront.setPower(0);
        drive.leftRear.setPower(0);
        drive.rightFront.setPower(0);
        drive.rightRear.setPower(0);
    }



}
