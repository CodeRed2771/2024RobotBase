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
    private CANSparkMax upperMotor;
    private CANSparkMax lowerMotor;
    private CANSparkMax loaderMotor;
    private AnalogInput loadSensor;
    private CANSparkMax intakeMotor;
    private BlinkinLED launcherLED;

    private double upperSpeedCmd = 0;
    private double lowerSpeedCmd = 0;
    private double speedTolerance = 0.05;
    private double motorSpeedBias = 0.06;

    private int notePresentThreshold = 1300; // < 1200 were starting to see a note
  
    private static final int STOP_DELAY = 2;
    private int loaderStopDelay = 0;
    public RollerLauncher(Map<String,Integer> wiring) {
        super();

        upperMotor = new CANSparkMax(wiring.get("upper launcher"), MotorType.kBrushless);
        lowerMotor = new CANSparkMax(wiring.get("lower launcher"), MotorType.kBrushless);
        loaderMotor = new CANSparkMax(wiring.get("launcher loader"), MotorType.kBrushless);

        loadSensor = new AnalogInput(wiring.get("load sensor"));

        int motorId = wiring.get("intakeMotorId");

        intakeMotor = new CANSparkMax(motorId, MotorType.kBrushless);

        launcherLED = new BlinkinLED(wiring.get("launcher led"));
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
    }

    public void prime(double power) {
        // in the future, set up so that the lower and upper motor power are set to a
        // slightly proportinal value to the
        // value fed into the function.
        upperSpeedCmd = -power;
        if (Math.abs(power)>.01)
            lowerSpeedCmd = (power) + motorSpeedBias;
        else  
            lowerSpeedCmd = 0;

        upperMotor.set(upperSpeedCmd);
        lowerMotor.set(lowerSpeedCmd);
    }

    public void setSpeedBias(double newBias) {
        motorSpeedBias = newBias;
    }

    public boolean isPrimed() {
        boolean upperMotorTracking = Math.abs(upperSpeedCmd - upperMotor.get()) < speedTolerance;
        boolean lowerMotorTracking = Math.abs(lowerSpeedCmd - lowerMotor.get()) < speedTolerance;

        return upperMotorTracking && lowerMotorTracking && Math.abs(upperSpeedCmd) > 0.1;
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
            if (loaderStopDelay == 0) {
                loaderMotor.set(0);
                loadState = LoaderState.Stopped;
            } else {
                loaderStopDelay--;
            }
        }

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
    }
}