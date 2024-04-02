// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmedSubsystem;

public abstract class NavSubsystem extends ArmedSubsystem {
  protected NavSubsystem() {
    super();
  }

  public abstract void reset(Pose2d init_pose);

  public void reset() {
    reset(new Pose2d());
  }

  public abstract Pose2d getPoseInField();

  public Translation2d getPosition() {
      return getPoseInField().getTranslation();
  }

  public Rotation2d getRotation(){
    return getPoseInField().getRotation();
  }

  public double getAngle() {
    return getRotation().getDegrees();
  }

}
