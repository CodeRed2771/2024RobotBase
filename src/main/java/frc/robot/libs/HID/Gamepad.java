package frc.robot.libs.HID;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * Contains functions for use with the Logitech F310 controller.
 *
 * @author art.kalb96@gmail.com (Arthur Kalb)
 * @author articgrayling8x8@gmail.com (Dorian Chan)
 * @author kevinsundar@gmail.com (Kevin Vincent)
 */

/**
 * Standalone class to make accessing a Logitech F310 gamepad simpler
 */

public class Gamepad extends XboxController {
    // Gamepad axis ports
    private static final int AXIS_LEFT_X = 1;
    private static final int AXIS_LEFT_Y = 2;
    private static final int AXIS_SHOULDER = 3;
    private static final int AXIS_RIGHT_X = 4;
    private static final int AXIS_RIGHT_Y = 5;
    private static final int AXIS_DPAD_X = 6;
    private static final int AXIS_DPAD_Y = 7;

    // Gamepad buttons
    private static final int BUTTON_A = 2;
    private static final int BUTTON_B = 3;
    private static final int BUTTON_X = 1;
    private static final int BUTTON_Y = 4;
    private static final int BUTTON_SHOULDER_LEFT = 5;
    private static final int BUTTON_SHOULDER_RIGHT = 6;
    private static final int BUTTON_TRIGGER_LEFT = 7;
    private static final int BUTTON_TRIGGER_RIGHT = 8;
    private static final int BUTTON_BACK = 9;
    private static final int BUTTON_START = 10;
    private static final int BUTTON_LEFT_STICK = 11;
    private static final int BUTTON_RIGHT_STICK = 12;
    private static final int BUTTON_MODE = -1;
    private static final int BUTTON_LOGITECH = -1;

    /**
     * Constructor that creates a Joystick object.
     */
    public Gamepad(int gamepadPort) {
        super(gamepadPort);
    }

    public double getDPadX() {
        return getRawAxis(AXIS_DPAD_X);
    }
    public double getDPadY() {
        return getRawAxis(AXIS_DPAD_Y);
    }


    /**
     * DPad Left and Right only WPILIB cannot access the vertical axis of the
     * Logitech Game Controller Dpad
     */

    // public boolean getDPadLeft() {
    //     double x = getDPadX();
    //     return (x < -0.5);
    // }

    // public boolean getDPadRight() {
    //     double x = getDPadX();
    //     return (x > 0.5);
    // }

    // public boolean getDPadUp() {
    //     double y = getDPadY();
    //     return (y < -0.5);
    // }

    // public boolean getDPadDown() {
    //     double y = getDPadY();
    //     return (y > 0.5);
    // }

    public boolean getDPadLeft() {
        int dPadValue = getPOV();
        return(dPadValue == 270) || (dPadValue == 315) || (dPadValue == 225);
    }
    public boolean getDPadRight() {
        int dPadValue = getPOV();
        return(dPadValue == 90) || (dPadValue == 135) || (dPadValue == 45);
    }
    public boolean getDPadUp() {
        int dPadValue = getPOV();
        return(dPadValue == 0) || (dPadValue == 45) || (dPadValue == 315);
    }
    public boolean getDPadDown() {
        int dPadValue = getPOV();
        return(dPadValue == 180) || (dPadValue == 225) || (dPadValue == 135);
    }

    public boolean getDPadLeftRestricted() {
        int dPadValue = getPOV();
        return(dPadValue == 270);
    }
    public boolean getDPadRightRestricted() {
        int dPadValue = getPOV();
        return(dPadValue == 90);
    }
    public boolean getDPadUpRestricted() {
        int dPadValue = getPOV();
        return(dPadValue == 0);
    }
    public boolean getDPadDownRestricted() {
        int dPadValue = getPOV();
        return(dPadValue == 180);
    }

    /**
     * Gets the state of the Start button
     * 
     * @return the state of the Start button
     */
    /**
     * Gets the state of the left shoulder
     * 
     * @return the state of the left shoulder
     */
    public JoystickButton getLeftShoulder() {
        return new JoystickButton(this, BUTTON_SHOULDER_LEFT);
    }

    /**
     * Gets the state of the right shoulder
     * 
     * @return the state of the right shoulder
     */
    public JoystickButton getRightShoulder() {
        return new JoystickButton(this, BUTTON_SHOULDER_RIGHT);
    }

    public JoystickButton getLeftStickClick() {
        return new JoystickButton(this, BUTTON_LEFT_STICK);
    }

    public JoystickButton getRightStickClick() {
        return new JoystickButton(this, BUTTON_RIGHT_STICK);
    }

    public JoystickButton getLeftTriggerClick() {
        return new JoystickButton(this, BUTTON_TRIGGER_LEFT);
    }

    public JoystickButton getRightTriggerClick() {
        return new JoystickButton(this, BUTTON_TRIGGER_RIGHT);
    }
}
