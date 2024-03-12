package frc.robot.subsystems.climber;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends ClimberSubsystem{
    private CANSparkMax liftMotor;
    // private CANSparkMax rightMotor;

    private RelativeEncoder liftEncoder;
    // private RelativeEncoder rightEncoder;

    private SparkPIDController PIDController;
    
    private final double MINIMUM_RETRACTION = 1;
    private final double MAXIMUM_EXTENTION = 115;
    private double lastPositionRequested = 0;

    public Climber(Map<String,Integer> wiring, Map<String,Double> calibration) {
        super();
        liftMotor = new CANSparkMax(wiring.get("climber"), MotorType.kBrushless);
        // leftMotor = new CANSparkMax(wiring.get("right climber"), MotorType.kBrushless);

        liftMotor.restoreFactoryDefaults();
        // rightMotor.restoreFactoryDefaults();

        liftMotor.setIdleMode(IdleMode.kBrake);
        // rightMotor.setIdleMode(IdleMode.kBrake);

        // rightMotor.follow(leftMotor);

        liftEncoder = liftMotor.getEncoder();
        liftEncoder.setPosition(0);
        // rightEncoder = rightMotor.getEncoder();

        PIDController = liftMotor.getPIDController();

        PIDController.setP(calibration.get("climber P"));
        PIDController.setI(calibration.get("climber I"));
        PIDController.setD(calibration.get("climber D"));
        PIDController.setIZone(calibration.get("climber Izone"));

        PIDController.setSmartMotionMaxVelocity(calibration.get("climber velocity"), 0);

        PIDController.setSmartMotionMinOutputVelocity(0, 0);

        PIDController.setSmartMotionMaxAccel(calibration.get("climber acceleration"), 0);

        PIDController.setSmartMotionAllowedClosedLoopError(1, 0);

        lastPositionRequested = 0;
    }

    @Override
    public void lift(double speed, boolean override) {
        double multiplier = 1;
        if(!override) {
            double requestedPos = lastPositionRequested + speed *multiplier;
            if(requestedPos > MAXIMUM_EXTENTION) {
                requestedPos = MAXIMUM_EXTENTION;
            } else if(requestedPos < MINIMUM_RETRACTION) {
                requestedPos = MINIMUM_RETRACTION;
            }
            PIDController.setReference(requestedPos, ControlType.kPosition);
            lastPositionRequested = requestedPos;
        } else {
            liftMotor.set(Math.signum(speed)*0.2);
        }
    }

    @Override
    public void reset() {
        super.reset();
        liftEncoder.setPosition(0);
        // rightEncoder.setPosition(0);
        lastPositionRequested = 0;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climb encoder", liftEncoder.getPosition());
        // SmartDashboard.putNumber("Current Time", System.currentTimeMillis());
        SmartDashboard.putNumber("Requested position", lastPositionRequested);
    }
}
