// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.ArmedSubsystem;

public abstract class SwerveModuleBase extends ArmedSubsystem {

  protected SwerveModuleBase() {
    super();
  }

  public abstract SwerveModulePosition getPosition();

  public abstract Rotation2d getRotation();

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public abstract SwerveModuleState getState();

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    var swerveState = getState();
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState targetState = SwerveModuleState.optimize(desiredState, swerveState.angle);

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired
    // direction of travel that can occur when modules change directions. This
    // results in smoother
    // driving.
    targetState.speedMetersPerSecond *= Math.pow(targetState.angle.minus(swerveState.angle).getCos(),1);

    applySwerveState(targetState);
  }

  protected abstract void applySwerveState(SwerveModuleState targetState);

}
