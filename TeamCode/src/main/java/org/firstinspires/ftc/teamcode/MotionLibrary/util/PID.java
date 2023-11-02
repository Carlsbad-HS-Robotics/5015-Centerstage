package org.firstinspires.ftc.teamcode.MotionLibrary.util;

/**
 * Class with built in methods for a PID feedforward loop
 * IMPORTANT: Make sure that the constants for each type are properly tuned and set before initialization of this class
 */
public class PID {
    public static double KPTurn;
    public static double KITurn;
    public static double KDTurn;

    public static double KPMove;
    public static double KIMove;
    public static double KDMove;

    public static double KPArm;
    public static double KIArm;
    public static double KDArm;
    public static double KPSlides;
    public static double KISlides;
    public static double KDSlides;

    public double KP;
    public double KI;
    public double KD;

    public double p;
    public double i;
    public double d;
    public double pidout = 0;

    NanoClock Timer = new NanoClock();
    public double updateTime = Timer.secondsLifeSpan();
    public double timeChange;
    public double lastUpdateTime = 0;
    public double lastError = 0;

    public static final int turn = 0;
    public static final int move = 1;


    /**
     * Initialize the PID object with a starting time and integral value of 0
     * @param type determines the constants used in this instance of the PID class.
     *             The constants can be inputted using the setConstants methods
     */
    public PID(Type type) {
        switch(type){
            case move:
            KP = KPMove;
            KI = KIMove;
            KD = KDMove;
            break;
            case turn:
            KP = KPTurn;
            KI = KITurn;
            KD = KDTurn;
            break;
            case arm:
            KP = KPArm;
            KI = KIArm;
            KD = KDArm;
            break;
            case slides:
            KP = KPSlides;
            KI = KISlides;
            KD = KDSlides;
            break;
        }
    }

    /**
     * Sets the static constants that can be used for controlling the arm
     * @param kp proportional multiplier constant
     * @param ki integral multiplier constant
     * @param kd derivative multiplier constant
     */
    public static void setConstantsArm(double kp, double ki, double kd) {
        KPArm = kp;
        KIArm = ki;
        KDArm = kd;
    }

    /**
     * Sets the static constants that can be used for controlling the turning
     * @param kp proportional multiplier constant
     * @param ki integral multiplier constant
     * @param kd derivative multiplier constant
     */
    public static void setConstantsTurn(double kp, double ki, double kd) {
        KPTurn = kp;
        KITurn = ki;
        KDTurn = kd;
    }

    /**
     * Sets the static constants that can be used for controlling the speed compared to distance
     * @param kp proportional multiplier constant
     * @param ki integral multiplier constant
     * @param kd derivative multiplier constant
     */
    public static void setConstantsMove(double kp, double ki, double kd) {
        KPMove = kp;
        KIMove = ki;
        KDMove = kd;
    }

    /**
     * Sets the static constants that can be used for controlling the speed compared to distance
     * @param kp proportional multiplier constant
     * @param ki integral multiplier constant
     * @param kd derivative multiplier constant
     */
    public static void setConstantsSlides(double kp, double ki, double kd) {
        KPSlides = kp;
        KISlides = ki;
        KDSlides = kd;
    }

    /**
     * Method for inputting
     * @param error
     */
    public void pid(double error) {

        updateTime = Timer.secondsLifeSpan();
        timeChange = updateTime - lastUpdateTime;
        if (timeChange != 0) {
            //P
            p = error * KP;
            //I
            i += timeChange * error * KI;
            //D
            d = (error - lastError) / timeChange * KD;

            pidout = p + i + d;
            lastUpdateTime = updateTime;
        }
    }

    /**
     * Resets the clock and all variables to 0 within this instance
     */
    public void reset() {
        p = 0;
        i = 0;
        d = 0;
        Timer.reset();
    }

    public enum Type {
        move,
        turn,
        arm,
        slides
    }
}
