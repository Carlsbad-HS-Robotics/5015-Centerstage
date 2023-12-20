//Cuttle Fish Docs: https://14496roboctopi.github.io/Cuttlefish/cuttlefish/index.html
//Three encoder LOC
CuttleEncoder leftEncoder ;
CuttleEncoder sideEncoder ;
CuttleEncoder rightEncoder;

ThreeEncoderLocalizer encoderLocalizer;

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
}
//The Localizer MUST be initialized AFTER the encoders are initialized and configured.
@Override
public void mainLoop()
{
        encoderLocalizer.update();
        System.out.println(encoderLocalizer.getPos());
        telemetry.addData("Localizer X:",encoderLocalizer.getPos().getX());
        telemetry.addData("Localizer Y:",encoderLocalizer.getPos().getY());
        telemetry.addData("Localizer R:",encoderLocalizer.getPos().getR());
        telemetry.update();
}