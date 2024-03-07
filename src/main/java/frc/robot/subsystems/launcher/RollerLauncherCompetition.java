// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.BlinkinLED;
import frc.robot.libs.BlinkinLED.LEDColors;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

/** Add your docs 
 * When load is command - run loader motor until note is in loader position
 * stop on sensor tells us we have note
 * hold then load for firing 
 * 
 * call prime to get motors up to speed 
 * 
 * Fire runs load for time if isPrimed until sensor says otherwise
 * 
 * 
 */
public class RollerLauncherCompetition extends LauncherSubsystem {
    private CANSparkFlex upperShooterMotor;
    private CANSparkFlex lowerShooterMotor;
    private CANSparkMax loaderMotor;
    private CANSparkMax aimMotor;
    private AnalogInput loadSensor;
    private CANSparkMax intakeMotor;
    private BlinkinLED launcherLED;

    private SparkPIDController upperPIDCtrl = null;
    private SparkPIDController lowerPIDCtrl = null;
    private SparkPIDController aimPIDController = null;
    
    private RelativeEncoder upperEncoder = null;
    private RelativeEncoder lowerEncoder = null;
    private RelativeEncoder aimEncoder = null;
    private DutyCycleEncoder aimAbsoluteEncoder = null;

    public double kP, kI, kD, kIz, kFF;
    public double aim_kP, aim_kI, aim_kD, aim_kIz, aim_kFF;
    
    public double kMaxOutput, kMinOutput, maxRPM;
    public double aim_kMaxOutput, aim_kMinOutput, aim_maxRPM;
    
    private double upperSpeedCmd = 0;
    private double lowerSpeedCmd = 0;
    private double speedTolerance = 300;
    private double motorSpeedBias = 0.06;

    private int notePresentThreshold = 1100; // < 1200 were starting to see a note

    private static final double ABS_FULL_BACK = .7205;
    private static final double ABS_TICK_BACK = .826902;
    private static final double ABS_FULL_FORWARD = 0.927;
    private static final double ABS_TICK_FORWARD = 266.9;
    private static final double AIM_RANGE_TICKS = ABS_TICK_FORWARD - ABS_TICK_BACK; // encoder ticks for full range of motion
    private static final double AIM_RANGE_FULL = ABS_FULL_FORWARD - ABS_FULL_BACK; // encoder ticks for full range of motion
    private double aimBias = 1.0;

    public double rollerRotationToTicks(double input_rotation){
        double rotations = aimBias - input_rotation / 360.0;
        double percent = (rotations - ABS_FULL_BACK)/AIM_RANGE_FULL;
        return ABS_TICK_BACK + percent * AIM_RANGE_TICKS;
    }

    public enum LauncherSpeeds {
        OFF(0,75),
        AMP(1300, 63.5),
        SUBWOOFER(2400, 71),
        SAFE_ZONE(2800, 48.5),
        STOW(0,25),
        MAX_ANGLE(0, 90);

        private double speed;
        private double angle;
        
        private LauncherSpeeds(double newSpeed, double newAngle) {
            speed = newSpeed;
            angle = newAngle;
        }

        public double getSpeed() {
            return speed;
        }
        public double getAngle() {
            // we want this to convert to encoder ticks
            return angle;
        }
    }
  
    private static final int STOP_DELAY = 100;
    private int loaderStopDelay = 0;
    private int loaderFireStopDelay = 0;

    public RollerLauncherCompetition(Map<String,Integer> wiring) {
        super();

        upperShooterMotor = new CANSparkFlex(wiring.get("upper launcher"), MotorType.kBrushless);
        lowerShooterMotor = new CANSparkFlex(wiring.get("lower launcher"), MotorType.kBrushless);
        loaderMotor = new CANSparkMax(wiring.get("launcher loader"), MotorType.kBrushless);
        aimMotor = new CANSparkMax(wiring.get("aim"), MotorType.kBrushless);

        loadSensor = new AnalogInput(wiring.get("load sensor"));

        int motorId = wiring.get("intakeMotorId");

        intakeMotor = new CANSparkMax(motorId, MotorType.kBrushless);

        launcherLED = new BlinkinLED(wiring.get("launcher led"));

        aimAbsoluteEncoder = new DutyCycleEncoder(wiring.get("aim encoder"));
        aimEncoder =  aimMotor.getEncoder();

        // Setup Shooter Motor Closed Loop Control
        upperShooterMotor.restoreFactoryDefaults();
        lowerShooterMotor.restoreFactoryDefaults();
        Timer.delay(0.5);

        upperShooterMotor.setInverted(false);
        lowerShooterMotor.setInverted(true);

        upperShooterMotor.burnFlash();
        lowerShooterMotor.burnFlash();
        Timer.delay(0.5);

        upperPIDCtrl = upperShooterMotor.getPIDController();
        lowerPIDCtrl = lowerShooterMotor.getPIDController();
        aimPIDController = aimMotor.getPIDController();

        upperEncoder = upperShooterMotor.getEncoder();
        lowerEncoder = lowerShooterMotor.getEncoder();

        kP = .0002 ; 
        kI = 0;
        kD = 0.01; 
        kIz = 0; 
        kFF = 0.000170; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        aim_kP = .08 ; 
        aim_kI = 0;
        aim_kD = 0; 
        aim_kIz = 0; 
        aim_kFF = 0.0; 
        aim_kMaxOutput = 1; 
        aim_kMinOutput = -1;

        // set PID coefficients
        upperPIDCtrl.setP(kP);
        upperPIDCtrl.setI(kI);
        upperPIDCtrl.setD(kD);
        upperPIDCtrl.setIZone(kIz);
        upperPIDCtrl.setFF(kFF);
        upperPIDCtrl.setOutputRange(kMinOutput, kMaxOutput);

        lowerPIDCtrl.setP(kP);
        lowerPIDCtrl.setI(kI);
        lowerPIDCtrl.setD(kD);
        lowerPIDCtrl.setIZone(kIz);
        lowerPIDCtrl.setFF(kFF);
        lowerPIDCtrl.setOutputRange(kMinOutput, kMaxOutput);

        resetAimEncoder();

        aimPIDController.setP(aim_kP);
        aimPIDController.setI(aim_kI);
        aimPIDController.setD(aim_kD);
        aimPIDController.setIZone(aim_kIz);
        aimPIDController.setFF(aim_kFF);
        aimPIDController.setOutputRange(aim_kMinOutput, aim_kMaxOutput);

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

    }

    private void resetAimEncoder() {
        aimEncoder.setPosition(rollerRotationToTicks(aimAbsoluteEncoder.get()));
    }

    private void log(String text) {
        System.out.println(getName() + " : " + text);
    }

    public void doArm() {
        log("Armed");
    }

    public void doDisarm() {
        log("Disarmed");
        stop();
        stopLoader();
    }

    public void load(double power) {
        super.load(power);

        loaderMotor.set(-power*2.00);
        intakeMotor.set(-power);
    }

    public void stopShooter() {
        prime(LauncherSpeeds.OFF);
    }

    public boolean isLoaded() {
        // value goes down as object gets closer to sensor
        SmartDashboard.putNumber("Note Sensor", loadSensor.getAverageValue());
        SmartDashboard.putBoolean("Is Loaded", loadSensor.getAverageValue() < notePresentThreshold);
        return loadSensor.getAverageValue() < notePresentThreshold;
    }

    public boolean isFiring() {
        return loadState == LoaderState.Firing;
    }

    public void fire() {
        if (isPrimed())
            {
            loadState = LoaderState.Firing;
            loaderFireStopDelay = STOP_DELAY;

            load(1); // run the loader which will put note into shooter
            }  
        else
            stopLoader();
    }

    public void unload() {
        super.unload();

        loaderMotor.set(.5);
        intakeMotor.set(.5);
        loadState = LoaderState.Unloading;
    }

    public void stopLoader() {
        super.stopLoader();
        loaderStopDelay = STOP_DELAY;

        intakeMotor.set(0);
        loaderMotor.set(0);
    }

    public void prime(LauncherSpeeds speedSetting) {
        double speed = speedSetting.getSpeed();
        double angleInTicks = rollerRotationToTicks(speedSetting.getAngle());

        // in the future, set up so that the lower and upper motor power are set to a
        // slightly proportinal value to the
        // value fed into the function.
        upperSpeedCmd = -speed;
        if (Math.abs(speed)>100)
            lowerSpeedCmd = (speed) *(1+motorSpeedBias);
        else  
            lowerSpeedCmd = 0;
        upperPIDCtrl.setReference(upperSpeedCmd, CANSparkMax.ControlType.kVelocity);
        lowerPIDCtrl.setReference(lowerSpeedCmd, CANSparkMax.ControlType.kVelocity);

        aimPIDController.setReference(angleInTicks, CANSparkMax.ControlType.kPosition);
    }

    public void setSpeedBias(double newBias) {
        motorSpeedBias = newBias;
    }

    public boolean isPrimed() {
        boolean upperMotorTracking = Math.abs(upperSpeedCmd - upperEncoder.getVelocity()) < speedTolerance;
        boolean lowerMotorTracking = Math.abs(lowerSpeedCmd - lowerEncoder.getVelocity()) < speedTolerance;

        return upperMotorTracking || lowerMotorTracking;
        // return true;
        // return upperMotorTracking && lowerMotorTracking && Math.abs(upperSpeedCmd) > 0.1;
    }

    public boolean isUnloading() {
        if(loadState == LoaderState.Unloading) {
            return true;
        }
        return false;
    }

    // public void aim(double angle) {
    //     double speed = subwoofer.get();
    //     // in the future, set up so that the lower and upper motor power are set to a
    //     // slightly proportinal value to the
    //     // value fed into the function.
    //     upperSpeedCmd = -speed;
    //     if (Math.abs(speed)>100)
    //         lowerSpeedCmd = (speed) *(1+motorSpeedBias);
    //     else  
    //         lowerSpeedCmd = 0;
    //     upperPIDCtrl.setReference(upperSpeedCmd, CANSparkMax.ControlType.kVelocity);
    //     lowerPIDCtrl.setReference(lowerSpeedCmd, CANSparkMax.ControlType.kVelocity);
    // }

    @Override
    public void periodic() {
        if (loadState == LoaderState.Stopping) {
            if (loaderStopDelay <= 0) {
                intakeMotor.set(0);
                loaderMotor.set(0);
                loadState = LoaderState.Stopped;
            } else {
                loaderStopDelay--;
            }
        }

        if (loaderFireStopDelay == 1) {
            stopShooter();
            intakeMotor.set(0);
            loaderMotor.set(0);
            loaderFireStopDelay--;
        } else 
            if (loaderFireStopDelay > 0)
            loaderFireStopDelay--;

        if(!isPrimed())
            launcherLED.blink(0.5);
        else
            launcherLED.blink(1);
        if(isLoaded())
            launcherLED.set(LEDColors.GREEN);
        else if(loadState == LoaderState.Loading)
            launcherLED.set(LEDColors.YELLOW);
        else
            launcherLED.set(LEDColors.RED);

        SmartDashboard.putNumber("FIRE STOP DELAY", loaderFireStopDelay);
        if (loadState == LoaderState.Firing) 
            SmartDashboard.putString("LOADER STATE", "FIRING");
        else if (loadState == LoaderState.Stopped) 
            SmartDashboard.putString("LOADER STATE", "STOPPED");
        else 
            SmartDashboard.putString("LOADER STATE","SOMETHING ELSE");
        
        
        SmartDashboard.putNumber("shooter speed", lowerShooterMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter SetPoint", lowerSpeedCmd);
                    // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { upperPIDCtrl.setP(p); lowerPIDCtrl.setP(p); kP = p; }
        if((i != kI)) { upperPIDCtrl.setI(i); lowerPIDCtrl.setI(i); kI = i; }
        if((d != kD)) { upperPIDCtrl.setD(d); lowerPIDCtrl.setD(d); kD = d; }
        if((iz != kIz)) { upperPIDCtrl.setIZone(iz); lowerPIDCtrl.setIZone(iz);kIz = iz; }
        if((ff != kFF)) { upperPIDCtrl.setFF(ff); lowerPIDCtrl.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            upperPIDCtrl.setOutputRange(min, max);
            lowerPIDCtrl.setOutputRange(min, max);  
            kMinOutput = min; kMaxOutput = max; 
        }

    SmartDashboard.putNumber("Aim Absolute Encoder", aimAbsoluteEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Aim Relative Encoder", aimEncoder.getPosition());
    }
}