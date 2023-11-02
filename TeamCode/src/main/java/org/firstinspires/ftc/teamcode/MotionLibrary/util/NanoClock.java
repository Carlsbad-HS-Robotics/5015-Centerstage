package org.firstinspires.ftc.teamcode.MotionLibrary.util;

/**
 * Clock and timer with functionality in nanoTime and seconds
 */
public class NanoClock extends Object {

    public NanoClock() {}

    private long clockStart = System.nanoTime();

    /**
     * Returns time that the system has been running for
     */
    public long totalTime(){
        return System.nanoTime();
    }

    /**
     * Returns the time since the NanoClock has been initialized in nanoseconds.
     * Can be used as a timer since the clock was last reset or initialised.
     */
    public long nanoLifespan() {
        return System.nanoTime() - clockStart;
    }

    /**
     * Returns the time since the NanoClock has been initialized in seconds.
     * Can be used as a timer since the clock was last reset or initialised.
     */
    public double secondsLifeSpan() {
        return nanoLifespan() / 1000000000;
    }

    /**
     * Resets the time that the NanoClock has been running to 0.
     * When used as a timer, this can be used to reset it to 0 again.
     */
    public void reset() {
        clockStart = System.nanoTime();
    }
}
