// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class PracticeRobotNav extends NavSubsystem {

    Translation2d position = new Translation2d();
    private NavXGyro gyro;

    public PracticeRobotNav() {
        super();

        gyro = new NavXGyro(SPI.Port.kMXP);
    }

    @Override
    public void reset() {
        gyro.reset();
    }

    @Override
    public double getAngle() {
        return gyro.getAngle();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Angle", ((int) (gyro.getAngle() * 1000)) / 1000.0);

        
    }

    @Override
    public Translation2d getPosition() {
        return position;
    }
}