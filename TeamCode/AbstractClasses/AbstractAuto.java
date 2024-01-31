package org.firstinspires.ftc.teamcode.AbstractClasses;

import java.io.IOException;

public abstract class AbstractAuto extends AbstractOpMode{

    public AbstractRobot robot;
    public abstract void autonomous();

    @Override
    public final void runOpMode() {

        super.runOpMode();
        robot = getRobot();

        try {

            robot.init();
            onInit();

            while(!isStarted() && !isStopRequested()) {}

            autonomous();

            robot.stop();
            super.onStop();
            super.stop();
            //super.stop();

        }
        catch (IllegalAccessException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}