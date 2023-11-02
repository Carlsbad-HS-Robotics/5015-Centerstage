package org.firstinspires.ftc.teamcode.MotionLibrary.util;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Can be used when an ultrasonic sensor is in the analog input port and you need to quickly convert that voltage to a distance.
 * Make sure that you tune the inchesPerVolt and the minimumVoltage.
 */
public class Ultrasonic {

    private AnalogInput sensor;

    /*
    How to tune inchesPerVolt and the minimumVoltage
    Go to a graphing calculator such as desmos and make a table
    Use the getVoltage() method to show the voltage outputted from the sensor
    Measure distances and put them in the X in the table and put the corresponding voltage in the Y in the table
    Make sure that you get both short and long distances in the table
    Create a line of best fit from the table
    The slope if the line is the inchesPerVolt and the y intercept is the minimumVoltage
     */

    private final double inchesPerVolt = 0.0113621;
    private final double minimumVoltage = 0.132718;

    /**
     * Constructor for Ultrasonic sensors
     * @param ultrasonic Input the sensor that you want to use as an analog input that is already initialized in the hardware map
     */
    public Ultrasonic(AnalogInput ultrasonic) {
        this.sensor = ultrasonic;
    }

    /**
     * Calculates the distance based on the constants and the current voltage of the ultrasonic sensor
     */
    public double getDistance() {
        return (sensor.getVoltage() - 0.1366) / 0.01191;
    }


    /**
     * Gets the voltage from the ultrasonic sensor.
     * Mainly used in tuning the sensor
     */
    public double getVoltage() {
        return sensor.getVoltage();
    }
}