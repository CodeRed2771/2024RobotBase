// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;

import edu.wpi.first.wpilibj.SPI;

/** Add your docs here. */
public class PracticeRobotNav extends NavSubsystem {

    private NavXGyro gyro;

    public PracticeRobotNav() {
        super();

        gyro = new NavXGyro(SPI.Port.kMXP);
    }

    @Override
    public void reset(){
        gyro.reset();
    }

    @Override
    public double getAngle() {
        return gyro.getAngle();
      }
}