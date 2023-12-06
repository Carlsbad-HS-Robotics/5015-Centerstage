package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.BlueDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="AutoBlueBackStage")
public class Auto extends LinearOpMode {

    SampleMecanumDrive drive;
    enum State{
        TRAJECTORY_1,
        DROP1,
        TRAJECTORY2,
        DROP2,
        TRAJECTORY3,
        IDLE
    }
    Arm arm_subsystem;
    OpenCvCamera webcam;
    BlueDetection.ObjectPosition position;
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
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        ElapsedTime timer = new ElapsedTime();

        drive = new SampleMecanumDrive(hardwareMap);

        Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                .back(30)
                .build();
        Trajectory backward = drive.trajectoryBuilder(new Pose2d())
                .forward(30)
                .build();
        Trajectory left = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(24)
                .build();
        drive.followTrajectoryAsync(forward);
        arm_subsystem = new Arm(hardwareMap);
        State currentState = State.IDLE;
        position = pipeline.getPosition();
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()) {
            switch(currentState){
                case TRAJECTORY_1:
                    arm_subsystem.grab();
                    if(!drive.isBusy()){
                        arm_subsystem.drop();
                        arm_subsystem.high();
                        timer.reset();
                        currentState = State.DROP1;
                    }
                    break;
                case DROP1:
                    if(timer.seconds() >= 2){

                        drive.followTrajectoryAsync(backward);
                        arm_subsystem.low();
                        currentState = State.TRAJECTORY2;   
                    }
                    break;
                case TRAJECTORY2:
                    if(!drive.isBusy()){
                        drive.followTrajectoryAsync(left);
                        arm_subsystem.grab();
                        currentState = State.DROP2;
                    }
                    break;
                case DROP2:
                    if(!drive.isBusy()){
                        arm_subsystem.drop();
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:

                    break;
            }

            //Code goes here:




            drive.update();
        }
    }





}
