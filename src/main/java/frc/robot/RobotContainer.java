// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.ArmedSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.nav.NavSubsystem;

public abstract class RobotContainer extends ArmedSubsystem {

  /* Be sure to register all subsystems after they are created */
  public DriveSubsystem drive;
  public IntakeSubsystem intake;
  public LauncherSubsystem launcher;
  public NavSubsystem nav;

  protected Map<String,Integer> wiring;

  protected RobotContainer() {
    super();

    wiring = new HashMap<>();
  }

  protected void registerSubsystems() {
    this.addChild(drive.getName(), drive);
    this.addChild(intake.getName(), intake);
    this.addChild(launcher.getName(), launcher);
    this.addChild(nav.getName(), nav);
  }

  public void restoreRobotToDefaultState(){}

  /* By default just pass commands to the drive system */
  public void driveSpeedControl(double fwd, double strafe, double rotate){
    drive.driveSpeedControl(fwd,strafe,rotate);
  }

  public void driveSpeedControlFieldCentric(double fwd, double strafe, double rotate) {
    /*
     * Rotate the drive command into field centric orientation by reversing out the
     * orientation of the robot
     */
    Translation2d command = new Translation2d(fwd, strafe);
    command = command.rotateBy(Rotation2d.fromDegrees(-nav.getAngle()));

    drive.driveSpeedControl(command.getX(), command.getY(), rotate);
  }

}
