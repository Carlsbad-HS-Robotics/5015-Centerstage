package org.firstinspires.ftc.teamcode;

import CuttleEncoder;
import CuttleMotor  ;

CuttleEncoder leftEncoder ;
CuttleEncoder sideEncoder ;
CuttleEncoder rightEncoder;

public CuttleMotor leftFrontMotor ;
public CuttleMotor rightFrontMotor;
public CuttleMotor rightBackMotor ;
public CuttleMotor leftBackMotor  ;

MecanumController chassis;

/*TODO: add arm_subsystem into a conditional list
 *      import hell :(
 */

@Autonomous(name="AutoRedBackstage", group="Centerstage")
public class cuttleAuto extends InitializedOpmode{

    PTPController ptpController;

    public final static double mm_per_inch = 25.4;

    @Override
    public void onInit()
    {
        //Drive Train
        leftFrontMotor  = ctrlHub.getMotor(1);
        rightFrontMotor = ctrlHub.getMotor(2);
        rightBackMotor  = ctrlHub.getMotor(3);
        leftBackMotor   = ctrlHub.getMotor(4);

        leftBackMotor .setDirection(Direction.REVERSE);
        leftFrontMotor.setDirection(Direction.REVERSE);

        chassis = new MecanumController(
            rightFrontMotor,
            rightBackMotor,
            leftFrontMotor,
            leftBackMotor
        );

        //Encoder
        leftEncoder  = expHub .getEncoder(3,720*4);
        sideEncoder  = ctrlHub.getEncoder(0,720*4);
        rightEncoder = ctrlHub.getEncoder(3,720*4);
        leftEncoder.setDirection(Direction.REVERSE);

        encoderLocalizer = new ThreeEncoderLocalizer(
                leftEncoder  ,
                sideEncoder  ,
                rightEncoder ,
                29,    // Radius of the wheel in mm
                130.5, // Distance between the two forward facing wheels in mm
                1.0    //Calibration constant 
        );

        //PID
        ptpController = new PTPController(chassis, encoderLocalizer);

        ptpController.setTranslational_PD_ctrlr(new PID(
            translation_p_gain,
            0,
            translation_d_gain,
            0.0,
            maximumIntegralPower));
        ptpController.setRotational_PID_ctrlr(new PID(
            rotational_p_gain,
            rotational_i_gain,
            rotational_d_gain,
            0.0,
            maximumIntegralPower));

        ptpController.setTranslational_PD_ctrlr(new PID( 
            0.02,  // Proportional
            0.0,   // Integral
            0.002, // Derivative
            0.0,   // Initial value (should be zero)
            1.0    // Maximum integral power (to prevent integral windup)
        ));
        ptpController.setRotational_PID_ctrlr(new PID(
            3,  // Proportional
            0.0,   // Integral
            0.2, // Derivative
            0.0,   // Initial value (should be zero)
            1.0    // Maximum integral power (to prevent integral windup)
        ));

        //Anti-Stall (continues to next WP if detects stalling)
        ptpController.getAntistallParams().setMovePowerAntistallThreshold(0.2); // Maxmimum translational power where the bot is still stalled
        ptpController.getAntistallParams().setRotatePowerAntistallThreshold(0.2); // Maxmimum rotation power where the bot is still stalled
        ptpController.getAntistallParams().setMoveSpeedAntistallThreshold(0.015);  // Maximum speed in m/s for the bot to be considered stalled
        ptpController.getAntistallParams().setRotateSpeedAntistallThreshold(0.3); // Maximum rotation speed in rad/s for the bot to be considered stalled
    }
        public void BackstageRed() {
        super.main();

        TaskList autoList = new TaskList();
        queue.addTask(new CustomTask(()->{
            switch(position){
                case LEFT:
                    arm_subsystem.grab();
                    autoList.addTask(new DelayTask(500));

                    autoList.addTask(new PointTask(
                        new Waypoint(
                            new Pose(32*mm_per_inch, 0, Math.PI/2),
                            0.5
                        ),
                        ptpController
                    ));

                    arm_subsystem.low();
                    arm_subsystem.update();

                    autoList.addTask(new DelayTask(1000));

                    arm_subsystem.release();
                    arm_subsystem.update();
                
            };
            
        }));

        queue.addTask(autoList);
}}