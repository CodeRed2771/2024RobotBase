// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;

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

    private double upperSpeedCmd = 0;
    private double lowerSpeedCmd = 0;
    private double speedTolerance = 0.05;
    private double motorSpeedBias = 0.06;

    private int notePresentFarThreshold = 1200; // < 1200 were starting to see a note
    private int notePresentCloseThreshold = 500; // < 500 indicates close to center of note

    public RollerLauncher(Map<String,Integer> wiring) {
        super();

        upperMotor = new CANSparkMax(wiring.get("upper launcher"), MotorType.kBrushless);
        lowerMotor = new CANSparkMax(wiring.get("lower launcher"), MotorType.kBrushless);
        loaderMotor = new CANSparkMax(wiring.get("launcher loader"), MotorType.kBrushless);

        loadSensor = new AnalogInput(wiring.get("load sensor"));
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

    public void load() {
        super.load();

        loaderMotor.set(-1);
    }

    public boolean isLoaded() {
        // value goes down as object gets closer to sensor
        return loadSensor.getAverageValue() < notePresentFarThreshold;
    }

    public void fire() {
        if (isLoaded() && isPrimed())
            load(); // run the loader which will put note into shooter
    }

    public void unload() {
        super.unload();

        loaderMotor.set(1);
    }

    public void stopLoader() {
        super.unload();

        loaderMotor.set(0);
    }

    public void prime(double power) {
        // in the future, set up so that the lower and upper motor power are set to a
        // slightly proportinal value to the
        // value fed into the function.
        upperSpeedCmd = -power;
        lowerSpeedCmd = (power) + motorSpeedBias;

        upperMotor.set(upperSpeedCmd);
        lowerMotor.set(lowerSpeedCmd);
    }

    public void setSpeedBias(double newBias) {
        motorSpeedBias = newBias;
    }

    public boolean isPrimed() {
        boolean upperMotorTracking = Math.abs(upperSpeedCmd - upperMotor.get()) < speedTolerance;
        boolean lowerMotorTracking = Math.abs(lowerSpeedCmd - lowerMotor.get()) < speedTolerance;

        return upperMotorTracking && lowerMotorTracking;
    }

    @Override
    public void periodic() {
        
    }
}