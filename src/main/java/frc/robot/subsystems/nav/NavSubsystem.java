// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class NavSubsystem extends SubsystemBase {
  protected NavSubsystem() {
    super();
  }

  public void reset() {
  }

  public abstract Translation2d getPosition();
  public abstract double getAngle();

}
