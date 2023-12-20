//Cuttle Fish Docs: https://14496roboctopi.github.io/Cuttlefish/cuttlefish/index.html

//Mecanum Setup
public CuttleMotor leftFrontMotor ;
public CuttleMotor rightFrontMotor;
public CuttleMotor rightBackMotor ;
public CuttleMotor leftBackMotor  ;
MecanumController chassis;

@Override
public void onInit(){
        leftFrontMotor  = ctrlHub.getMotor(3);
        rightFrontMotor = ctrlHub.getMotor(2);
        rightBackMotor  = expHub .getMotor(2);
        leftBackMotor   = expHub .getMotor(3);

        leftBackMotor .setDirection(Direction.REVERSE);
        leftFrontMotor.setDirection(Direction.REVERSE);

        chassis = new MecanumController(rightFrontMotor,rightBackMotor,leftFrontMotor,leftBackMotor);
}

public void mainLoop(){
    chassis.setVec(new Pose(gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x));
}

//WAYPOINTS!!!!

public class InitializedOpmode extends GamepadOpMode 
{        
    PTPController ptpController
    public void onInit()
    {
        //Init Mecanum Controller Here
        //Init Localizer Here
        ptpController = new PTPController(chassis, encoderLocalizer);
    }
}

queue.addTask(new PointTask(
    new Waypoint(
        new Pose(1000,0,0),
        0.5 // Maximum power
    ),
    ptpController
)); 

/*tuning?
There are several parameters that can be tuned in the Point to Point controller. 
These are the PID Controllers, and the antistall system. 
There is two PID controllers. First is the rotational PID controller, 
which controls the angle of the bot, and second the translational 
PD controller which controls the position of the bot. 
The translational PD controlled does not have a signed error 
meaning that the I gain must be set to zero. 
Here is how you set the controller coefficients:*/

ptpController.setTranslational_PD_ctrlr(new PID(translation_p_gain,0,translation_d_gain,0.0,maximumIntegralPower));
ptpController.setRotational_PID_ctrlr(new PID(rotational_p_gain,rotational_i_gain,rotational_d_gain,0.0,maximumIntegralPower));

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

ptpController.getAntistallParams().setMovePowerAntistallThreshold(0.2); // Maxmimum translational power where the bot is still stalled
ptpController.getAntistallParams().setRotatePowerAntistallThreshold(0.2); // Maxmimum rotation power where the bot is still stalled
ptpController.getAntistallParams().setMoveSpeedAntistallThreshold(0.015);  // Maximum speed in m/s for the bot to be considered stalled
ptpController.getAntistallParams().setRotateSpeedAntistallThreshold(0.3); // Maximum rotation speed in rad/s for the bot to be considered stalled

//TASK QUEUE!!!!!
public class InitializedOpmode extends GamepadOpMode {
        TaskQueue queue = new TaskQueue();
        public void onInit(){

        }
}

//Task Queue Example
@TeleOp(name="Queue Example", group="Cuttle Fish")
public class QueueExample extends InitializedOpmode {
    
    public void onInit(){
        super.onInit();
    }

    public void main(){
        super.main();

        // Go forward 1000mm
        queue.addTask(new PointTask(
                new Waypoint(
                        new Pose(1000,0,0),
                        0.5
                ),
                ptpController
        )); 

        // Go sideways 1000mm and turn 0.5PI Radians (90 degrees)
        queue.addTask(new PointTask(
                new Waypoint(
                        new Pose(1000,1000,Math.PI/2),
                        0.5
                ),
                ptpController
        )); 

        // Open the claw 
        queue.addTask(new ServoTask(claw_servo,CLAW_OPEN));
        
        //Delay to make sure the servo has time to move
        queue.addTask(new DelayTask(400));

        //Drive back to the starting position
        queue.addTask(new PointTask(
                new Waypoint(
                        new Pose(0,0,0),
                        0.5
                ),
                ptpController
        ));
        
    }
    public void mainLoop()
    {
        super.mainLoop();
    }
}