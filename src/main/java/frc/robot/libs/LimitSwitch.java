package frc.robot.libs;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch {
    private DigitalInput switchInput;
    private boolean isInverted = false;

    /**
     * 
     * @param channel specifies the digital input channel to use for reading the switch state.
     */
    public LimitSwitch(int channel) {
        switchInput = new DigitalInput(channel);
    }
    /**
     * 
     * @param channel specifies the digital input channel to use for reading the switch state.
     * @param isInverted whether the signal is inverted when being read
     */
    public LimitSwitch(int channel,boolean isInverted) {
        switchInput = new DigitalInput(channel);
        this.isInverted = isInverted;
    }

    /**
     * 
     * @return returns true if switch is pressed
     */
    public boolean isPressed() {
        if(isInverted){
            return switchInput.get(); // Returns true if the switch is pressed (input is low)
        } else {
            return !switchInput.get(); // Returns true if the switch is pressed (input is low)
        }
    }

    /**
     * 
     * @return boolean whether the output is inverted or not
     */
    public boolean isInverted() {
        return isInverted;
    }
    /**
     * 
     * @param invert set whether the output is inverted or not
     */
    public void invert(boolean invert) {
        this.isInverted = invert;
    }
}

