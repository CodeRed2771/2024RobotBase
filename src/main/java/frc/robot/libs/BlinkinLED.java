package frc.robot.libs;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkinLED extends SubsystemBase {
  
  public enum LEDColors {
    RED(0.805),
    YELLOW(0.845),
    GREEN(0.885),
    BLUE(0.935),
    VIOLET(0.955),
    WHITE(0.965),
    OFF(1);

    private double value;

    private LEDColors(double newValue) {
      value = newValue;
    }

    public double get() {
      return value;
    }
  }
  private PWM led;
  private LEDColors currentColor;
  private int blinkTimer = 0;
  private int blinkCycles = 0;
  private static final int MAXCYCLES = 50;

  public BlinkinLED(int channel) {
    super();
    led = new PWM(channel);
    set(LEDColors.OFF);
    blink(1);
  }

  public void set(LEDColors color) {
    currentColor = color;
    led.setPosition(color.get());
  }

  public void blink(double rate) {
    //rate between 0 and 1
    if(rate < 0)
      rate = 0;
    if(rate > 1)
      rate = 1;
    blinkTimer = (int) (rate*MAXCYCLES);
  }

  @Override
  public void periodic() {
    // TODO: Replace with Timer.getFPGATimestamp()
    if(blinkCycles < MAXCYCLES)
      blinkCycles++;
    else
      blinkCycles = 0;

    if(blinkCycles < blinkTimer)
      led.setPosition(currentColor.get());
    else
      led.setPosition(LEDColors.OFF.get());
  }
}