// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import java.util.Map;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.BlinkinLED;
import frc.robot.libs.BlinkinLED.LEDColors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

/**
 * Add your docs When load is command - run loader motor until note is in loader position stop on sensor tells us we
 * have note hold then load for firing call prime to get motors up to speed Fire runs load for time if isPrimed until
 * sensor says otherwise
 */
public class RollerLauncher extends LauncherSubsystem {
    protected CANSparkFlex upperShooterMotor;
    protected CANSparkFlex lowerShooterMotor;
    protected CANSparkMax loaderMotor;
    protected AnalogInput loadSensor;
    protected CANSparkMax intakeMotor;
    protected BlinkinLED launcherLED;

    protected SparkPIDController upperPIDCtrl = null;
    protected SparkPIDController lowerPIDCtrl = null;

    protected RelativeEncoder upperEncoder = null;
    protected RelativeEncoder lowerEncoder = null;

    public double kP, kI, kD, kIz, kFF;

    public double kMaxOutput, kMinOutput, maxRPM;

    private double upperSpeedCmd = 0;
    private double lowerSpeedCmd = 0;
    private double speedTolerance = 300;
    private double motorSpeedBias = 0.06;
    private double upperDirection = 1.0;
    private double lowerDirection = 1.0;

    protected int notePresentThreshold;

    private int STOP_DELAY;
    private int loaderStopDelay = 0;
    private int loaderFireStopDelay = 0;

    public RollerLauncher(Map<String, Integer> wiring, Map<String, Double> calibration) {
        super();

        notePresentThreshold = calibration.getOrDefault("note threshold", 1700.0).intValue();
        STOP_DELAY = calibration.getOrDefault("fire stop delay", 100.0).intValue();

        upperShooterMotor = new CANSparkFlex(wiring.get("upper launcher"), MotorType.kBrushless);
        lowerShooterMotor = new CANSparkFlex(wiring.get("lower launcher"), MotorType.kBrushless);
        loaderMotor = new CANSparkMax(wiring.get("launcher loader"), MotorType.kBrushless);

        loadSensor = new AnalogInput(wiring.get("load sensor"));
        loadState = LoaderState.Stopped;

        int motorId = wiring.get("intakeMotorId");

        intakeMotor = new CANSparkMax(motorId, MotorType.kBrushless);
        intakeMotor.setInverted(true);

        launcherLED = new BlinkinLED(wiring.get("launcher led"));

        // Setup Shooter Motor Closed Loop Control
        upperShooterMotor.restoreFactoryDefaults();
        lowerShooterMotor.restoreFactoryDefaults();
        Timer.delay(0.5);

        upperDirection = calibration.getOrDefault("upper launcher direction", -1.0);
        upperShooterMotor.setInverted(upperDirection < 0);
        lowerDirection = calibration.getOrDefault("lower launcher direction", -1.0);
        lowerShooterMotor.setInverted(lowerDirection < 0);
        upperShooterMotor.setIdleMode(IdleMode.kCoast);
        lowerShooterMotor.setIdleMode(IdleMode.kCoast);

        upperPIDCtrl = upperShooterMotor.getPIDController();
        lowerPIDCtrl = lowerShooterMotor.getPIDController();

        upperEncoder = upperShooterMotor.getEncoder();
        lowerEncoder = lowerShooterMotor.getEncoder();

        kP = .0014;
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

        upperShooterMotor.burnFlash();
        lowerShooterMotor.burnFlash();
        Timer.delay(0.5);

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

    @Override
    protected void doArm() {
        log("Armed");
    }

    @Override
    protected void doDisarm() {
        log("Disarmed");
        stop();
        stopLoader();
    }

    @Override
    public void load(double power) {
        super.load(power);

        loaderMotor.set(power * 1.10);
        intakeMotor.set(-power);
    }

    public void stopShooter() {
        prime(0);
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
        loaderFireStopDelay = STOP_DELAY;
        load(1); // run the loader which will put note into shooter

        super.fire();
    }

    public void unload() {
        super.unload();

        loaderMotor.set(-.5);
        intakeMotor.set(.5);
        loadState = LoaderState.Unloading;
    }

    public void stopLoader() {
        super.stopLoader();

        intakeMotor.set(0);
        loaderMotor.set(0);
    }

    public void stopFireDelay() {
        stopLoader();
        loaderFireStopDelay = 0;
    }

    public void prime(double speed, double bias) {
        prime(speed);
        setSpeedBias(bias);
    }

    @Override
    public void prime(double speed) {

        // in the future, set up so that the lower and upper motor power are set to a
        // slightly proportinal value to the
        // value fed into the function.
        upperSpeedCmd = -speed;
        if (Math.abs(speed) > 100)
            lowerSpeedCmd = (speed) * (1 + motorSpeedBias);
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
        if (loadState == LoaderState.Unloading) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        if (loadState == LoaderState.Stopping) {
            if (loaderStopDelay <= 0) {
                stopLoader();
            } else {
                loaderStopDelay--;
            }
        }

        if (loaderFireStopDelay == 1) {
            stopShooter();
            stopLoader();

            loaderFireStopDelay--;
        } else if (loaderFireStopDelay > 0)
            loaderFireStopDelay--;

        if (isLoaded())
            launcherLED.set(LEDColors.GREEN);
        else if (loadState == LoaderState.Loading || loadState == LoaderState.Unloading)
            launcherLED.set(LEDColors.YELLOW);
        else
            launcherLED.set(LEDColors.RED);

        SmartDashboard.putNumber("FIRE STOP DELAY", loaderFireStopDelay);
        SmartDashboard.putString("LOADER STATE", loadState.toString());

        SmartDashboard.putNumber("shooter bias", motorSpeedBias);
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
        if ((p != kP)) {
            upperPIDCtrl.setP(p);
            lowerPIDCtrl.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            upperPIDCtrl.setI(i);
            lowerPIDCtrl.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            upperPIDCtrl.setD(d);
            lowerPIDCtrl.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            upperPIDCtrl.setIZone(iz);
            lowerPIDCtrl.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            upperPIDCtrl.setFF(ff);
            lowerPIDCtrl.setFF(ff);
            kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            upperPIDCtrl.setOutputRange(min, max);
            lowerPIDCtrl.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }
    }
}