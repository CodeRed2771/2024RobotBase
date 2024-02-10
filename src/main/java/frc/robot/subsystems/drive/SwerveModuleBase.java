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

  // Gains are zero'd in abstract class. Update in particular Swerve constructor
  protected PIDController m_drivePIDController;
  protected ProfiledPIDController m_turningPIDController;
  protected SimpleMotorFeedforward m_driveFeedforward;
  protected SimpleMotorFeedforward m_turnFeedforward;

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
    targetState.speedMetersPerSecond *= targetState.angle.minus(swerveState.angle).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(swerveState.speedMetersPerSecond,
        targetState.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(targetState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(swerveState.angle.getRadians(),
        targetState.angle.getRadians());

    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    commandSwerveModuleState(driveOutput + driveFeedforward, turnOutput + turnFeedforward);
  }

  protected abstract void commandSwerveModuleState(double driveCmd, double turnCmd);

}
