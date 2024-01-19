package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="AutoBlue(real)")
public class Auto3 extends LinearOpMode {
    Definitions drive;
    //SampleMecanumDrive drive;
    enum State{
        TRAJECTORY_1,
        DROP1,
        TRAJECTORY2,
        DROP2,
        TRAJECTORY3,
        IDLE
    }
    Arm arm_subsystem;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        drive = new Definitions(hardwareMap);

        /*
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

         */
        waitForStart();

        drive.rightFront.set(-1);
        drive.leftFront.set(1);
        drive.rightRear.set(1);
        drive.leftRear.set(-1);

        sleep(1000);

        drive.rightFront.set(0);
        drive.leftFront.set(0);
        drive.rightRear.set(0);
        drive.leftRear.set(0);

        if(isStopRequested()) return;
        while(opModeIsActive()) {


            /*switch(currentState){
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


                        arm_subsystem.low();
                        currentState = State.IDLE;
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
            }*/



            //Code goes here:




            //drive.update();
        }
    }
   // void forward(5000)





}
