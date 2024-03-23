package frc.robot.subsystems.climber;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmedSubsystem;

public abstract class ClimberSubsystem extends ArmedSubsystem {
    protected ClimberSubsystem() {
        super();
    }
    public void lift(double speed, boolean override){}
    public void releaseClutch() {}
    public void engageClutch() {}
    public void reset() {}
}
