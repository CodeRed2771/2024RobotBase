// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Map;

/** Add your docs here. */
public class TankDriveSubsystem extends DriveSubsystem {

    TankWheelVortex leftDrive;
    TankWheelVortex rightDrive;

    double kMaxSpeed = 1.0;

    public TankDriveSubsystem(Map<String,Integer> wiring, Map<String,Double> calibration) {
        super();

        leftDrive = new TankWheelVortex(wiring, calibration, "A");
        rightDrive = new TankWheelVortex(wiring, calibration, "B");

        this.addChild(leftDrive.getName(), leftDrive);
        this.addChild(rightDrive.getName(), rightDrive);
    }

    @Override
    public void driveSpeedControl(double fwd, double strafe, double rot){
        // Motors are mounted in opposite orientation from each other
        leftDrive.commandSpeed(fwd - rot);
        rightDrive.commandSpeed(-fwd - rot);
    }
}
