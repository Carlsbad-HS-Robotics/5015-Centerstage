CuttleEncoder leftEncoder ;
CuttleEncoder sideEncoder ;
CuttleEncoder rightEncoder;

public CuttleMotor leftFrontMotor ;
public CuttleMotor rightFrontMotor;
public CuttleMotor rightBackMotor ;
public CuttleMotor leftBackMotor  ;

MecanumController chassis;

@Autonomous(name="AutoRedBackstage", group="Centerstage")
public class cuttleAuto extends InitializedOpmode{
    @Override
    public void onInit()
    {
            leftEncoder  = expHub .getEncoder(3,720*4);
            sideEncoder  = ctrlHub.getEncoder(0,720*4);
            rightEncoder = ctrlHub.getEncoder(3,720*4);
            leftEncoder.setDirection(Direction.REVERSE);

            encoderLocalizer = new ThreeEncoderLocalizer(
                    leftEncoder  ,
                    sideEncoder  ,
                    rightEncoder ,
                    29, // Radius of the wheel in mm
                    130.5, // Distance between the two forward facing wheels in mm
                    1.0 //Calibration constant (see below)
            );

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
    }
    
    public void main{
        super.main();

        queue.addTask(new PointTask(
            new Waypoint(
                new Pose(1000,0,0),
                0.5
            ),
            ptpController
        ));

        queue.addTask(new PointTask(
            new Waypoint(
                new Pose(1000,1000,Math.PI/2),
                0.5
            ),
            ptpController
        ));

        queue.addTask(new ServoTask(claw, CLAW_OPEN));

        queue.addTask(new DelayTask(400));

        queue.addTask(new PointTask(
            new Waypoint(
                new Pose(0,0,0),
                0.5
            ),
            ptpController
        ));
    }

    public void mainLoop(){
        super.mainLoop()
    }
}
