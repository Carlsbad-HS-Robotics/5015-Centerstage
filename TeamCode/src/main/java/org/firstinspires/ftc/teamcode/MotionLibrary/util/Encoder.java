package org.firstinspires.ftc.teamcode.MotionLibrary.util;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Class that can allow you to change the direction of an encoder without changing the direction of the motor in the corresponding port.
 * Useful for encoders that are separate from the motors such as dead wheels
 */
public class Encoder extends Object {

    private double velocityEstimate;
    private final double timeLastUpdate;
    private DcMotorEx motor;
    private double lastPosition;
    private Direction direction;
    private double gearRatio = 1;
    private double encoderTicsPerInch = 1;
    private NanoClock clock = new NanoClock();

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    /**
     * Calculates the multiplier based on the given direction and ratio inputs
     */
    private double getMultiplier() {
        return gearRatio * this.direction.multiplier / encoderTicsPerInch;
    }

    public Encoder(DcMotorEx motor, double GearRatio, double EncoderTicsPerInch){
        this.motor = motor;

        this.direction = Direction.FORWARD;
        this.lastPosition = 0;
        this.velocityEstimate = 0;
        this.timeLastUpdate = 0;
        gearRatio = GearRatio;
        encoderTicsPerInch = EncoderTicsPerInch;
    }

    public Encoder(DcMotorEx motor, double GearRatio){
        this.motor = motor;

        this.direction = Direction.FORWARD;
        this.lastPosition = 0;
        this.velocityEstimate = 0;
        this.timeLastUpdate = 0;
        gearRatio = GearRatio;
    }

    /**
     * Sets the direction that this encoder is pointing
     */
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    /**
     * Sets the gear ratio for this encoder
     */
    public void setGearRatio(int gearratio) {
        this.gearRatio = gearratio;
    }

    public Direction getDirection() {
        return direction;
    }

    private double lastUpdateTime;

    /**
     * Gets current position for use when the encoder is measuring the direction moved on the field
     */
    public double getCurrentPosition() {
        double multiplier = getMultiplier();
        double currentPosition = motor.getCurrentPosition() * multiplier;
        if (currentPosition != lastPosition) {
            double currentTime = clock.nanoLifespan();
            double dt = currentTime - lastUpdateTime;
            velocityEstimate = (currentPosition - lastPosition) / dt;
            lastPosition = currentPosition;
            lastUpdateTime = currentTime;
        }
        return currentPosition;
    }

    /**
     * Returns the degrees that the encoder has measured.
     * For use when the encoder is measuring just a rotation.
     * Ex: Encoder to measure the rotation of an arm
     */
    public double getDegrees() {
        double multiplier = gearRatio;
        double currentPosition = motor.getCurrentPosition() * multiplier;
        if (currentPosition != lastPosition) {
            double currentTime = clock.nanoLifespan();
            double dt = currentTime - lastUpdateTime;
            velocityEstimate = (currentPosition - lastPosition) / dt;
            lastPosition = currentPosition;
            lastUpdateTime = currentTime;
        }
        return currentPosition;
    }

    /**
     * Resets the values that the encoder has measured back to 0
     */
    public void reset(){
        motor.setMode(STOP_AND_RESET_ENCODER);
    }
}