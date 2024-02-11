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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "BLueBackstage")
public class AutoBlueBackstage extends LinearOpMode {
    enum State {
        FORWARD,
        LEFT,
        MIDDLE,
        RIGHT,
        DROP1,
        TURN,
        FORWARD2,
        DROP2,
        IDLE
    }

    SampleMecanumDrive drive;
    Arm arm_subsystem;
    OpenCvCamera webcam;
    volatile BlueDetection.ObjectPosition position;
    @Override
    public void runOpMode() throws InterruptedException {


        drive = new SampleMecanumDrive(hardwareMap);
        // TODO fix spelling
        State currentState = State.IDLE;

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



        Pose2d startPose = new Pose2d(0, 0, 0);
        ElapsedTime waitTimer1 = new ElapsedTime();
        Trajectory forward = drive.trajectoryBuilder(startPose)
                .back(36)
                .build();
        Trajectory forward2 = drive.trajectoryBuilder(forward.end())
                .back(30)
                .build();
        Trajectory left = drive.trajectoryBuilder(forward.end())
                .strafeRight(10)
                .build();
        Trajectory middle = drive.trajectoryBuilder(forward.end())
                .back(6)
                .build();
        Trajectory right = drive.trajectoryBuilder(forward.end())
                .strafeLeft(10)
                .build();
        currentState = State.FORWARD; //TODO: change this to the pipeline
        drive.followTrajectoryAsync(forward);
        waitForStart();
        position = pipeline.getPosition();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            switch (currentState) {
                case FORWARD:
                    if (!drive.isBusy()) {
                        if(position == BlueDetection.ObjectPosition.LEFT){
                            currentState = State.LEFT;
                            drive.followTrajectoryAsync(left);
                        } else if (position == BlueDetection.ObjectPosition.CENTER){
                            currentState = State.LEFT;
                            drive.followTrajectoryAsync(middle);
                        } else {
                            currentState = State.LEFT;
                            drive.followTrajectoryAsync(right);
                        }

                    }
                    break;
                case LEFT:
                    if(!drive.isBusy()){
                        currentState = State.DROP1;
                        //TODO: add arm drop

                        waitTimer1.reset();
                    }
                    break;
                case DROP1:
                    if (waitTimer1.seconds() > 2) {
                        currentState = State.TURN;
                        drive.turn(Math.toRadians(90));
                    }
                    break;
                case TURN:
                    if (!drive.isBusy()) {
                        currentState = State.FORWARD2;
                        drive.followTrajectoryAsync(forward2);
                    }
                    break;
                case FORWARD2:
                    if(!drive.isBusy()) {
                        currentState = State.DROP2;
                        //TODO: add arm drop
                        waitTimer1.reset();
                    }
                case DROP2:
                    if(waitTimer1.seconds() > 2){
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            drive.update();
        }
    }
}
