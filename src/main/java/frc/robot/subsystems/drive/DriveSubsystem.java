// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import frc.robot.subsystems.ArmedSubsystem;

public abstract class DriveSubsystem extends ArmedSubsystem {
  protected DriveSubsystem() {
    super();
  }

  public void reset(){}

  public void driveSpeedControl(double fwd, double strafe, double rot){}

  public void driveInches(double inches, double speedFactor){}

  public void driveInches(double inches, double speedFactor, double turnAngle){}

  public void rotateDegrees(double degrees, double speedFactor){}

  public void strafeInches(double inches, double speedFactor){}
}
