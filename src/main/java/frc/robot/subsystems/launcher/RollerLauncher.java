// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public class RollerLauncher extends LauncherSubsystem {
    private CANSparkMax upperMotor;
    private CANSparkMax lowerMotor;

    private double upperSpeedCmd = 0;
    private double lowerSpeedCmd = 0;
    private double speedTolerance = 0.05;
    private double motorSpeedBias = 0.06;


    public RollerLauncher(int upperMotorId, int lowerMotorId) {
        super();

        upperMotor = new CANSparkMax(upperMotorId, MotorType.kBrushless);
        lowerMotor = new CANSparkMax(lowerMotorId, MotorType.kBrushless);
    }

    private void log(String text) {
        System.out.println(getName() + " : " + text);
    }

    public void doArm() {
        log("Armed");
    }

    public void doDisarm() {
        log("Disarmed");
    }

    public void load() {
        log("Running load actuators");
    }

    public void prime(double power) {
        // in the future, set up so that the lower and upper motor power are set to a
        // slightly proportinal value to the
        // value fed into the function.
        upperSpeedCmd = power;
        lowerSpeedCmd = (-power) + motorSpeedBias;

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

}