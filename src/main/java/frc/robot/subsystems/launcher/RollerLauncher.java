// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.BlinkinLED;
import frc.robot.libs.BlinkinLED.LEDColors;

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
public class RollerLauncher extends LauncherSubsystem {
    private CANSparkMax upperShooterMotor;
    private CANSparkMax lowerShooterMotor;
    private CANSparkMax loaderMotor;
    private AnalogInput loadSensor;
    private CANSparkMax intakeMotor;
    private BlinkinLED launcherLED;

    private SparkPIDController upperPIDCtrl = null;
    private SparkPIDController lowerPIDCtrl = null;
    
    private RelativeEncoder upperEncoder = null;
    private RelativeEncoder lowerEncoder = null;

    public double kP, kI, kD, kIz, kFF;
    
    public double kMaxOutput, kMinOutput, maxRPM;
    
    private double upperSpeedCmd = 0;
    private double lowerSpeedCmd = 0;
    private double speedTolerance = 300;
    private double motorSpeedBias = 0.06;

    private int notePresentThreshold = 1800; // < 1200 were starting to see a note

    public enum LauncherSpeeds {
        OFF(0),
        AMP(1000),
        SUBWOOFER(2200), // tested 2/22/24
        SAFE_ZONE(2700); // untested

        private double speed;
        
        private LauncherSpeeds(double newSpeed) {
            speed = newSpeed;
        }

        public double get() {
            return speed;
        }
    }
  
    private static final int STOP_DELAY = 100;
    private int loaderStopDelay = 0;
    private int loaderFireStopDelay = 0;

    public RollerLauncher(Map<String,Integer> wiring) {
        super();

        upperShooterMotor = new CANSparkMax(wiring.get("upper launcher"), MotorType.kBrushless);
        lowerShooterMotor = new CANSparkMax(wiring.get("lower launcher"), MotorType.kBrushless);
        loaderMotor = new CANSparkMax(wiring.get("launcher loader"), MotorType.kBrushless);

        loadSensor = new AnalogInput(wiring.get("load sensor"));

        int motorId = wiring.get("intakeMotorId");

        intakeMotor = new CANSparkMax(motorId, MotorType.kBrushless);

        launcherLED = new BlinkinLED(wiring.get("launcher led"));

        // Setup Shooter Motor Closed Loop Control
        upperShooterMotor.restoreFactoryDefaults();
        lowerShooterMotor.restoreFactoryDefaults();

        upperShooterMotor.setInverted(true);
        lowerShooterMotor.setInverted(true);

        upperPIDCtrl = upperShooterMotor.getPIDController();
        lowerPIDCtrl = lowerShooterMotor.getPIDController();

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

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
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

        loaderMotor.set(power*1.10);
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

        loaderMotor.set(-.5);
        intakeMotor.set(.5);
        loadState = LoaderState.Unloading;
    }

    public void stopLoader() {
        super.stopLoader();
        loaderStopDelay = STOP_DELAY;

        intakeMotor.set(0);
        loaderMotor.set(0);
    }

    public void prime(LauncherSpeeds commandSpeed) {
        double speed = commandSpeed.get();
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
    }

    public void setSpeedBias(double newBias) {
        motorSpeedBias = newBias;
    }

    public boolean isPrimed() {
        boolean upperMotorTracking = Math.abs(upperSpeedCmd - upperEncoder.getVelocity()) < speedTolerance;
        boolean lowerMotorTracking = Math.abs(lowerSpeedCmd - lowerEncoder.getVelocity()) < speedTolerance;

        return upperMotorTracking || lowerMotorTracking;
        // return upperMotorTracking && lowerMotorTracking && Math.abs(upperSpeedCmd) > 0.1;
    }

    public boolean isUnloading() {
        if(loadState == LoaderState.Unloading) {
            return true;
        }
        return false;
    }

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
    }
}