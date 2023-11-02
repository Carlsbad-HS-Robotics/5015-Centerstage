package org.firstinspires.ftc.teamcode.MotionLibrary.util;

/**
 * A class that can be used when making a button toggle something.
 * It will stop the toggle from switching states every loop while the button is held down.
 */
public class Toggle {
    public boolean heldLastFrame = false;
    public boolean state;
    public boolean needChange = false;

    public Toggle(boolean state) {
        this.state = state;
    }

    /**
     * Updates the state of the toggle only if the state was not held the last frame.
     * @param input input the state from the button that you want to affect this toggle
     */
    public void update(boolean input) {
        if (!heldLastFrame && input) {
            state = !state;
            heldLastFrame = true;
            needChange = true;
        }

        if (input) heldLastFrame = true;
        else heldLastFrame = false;
    }
}
