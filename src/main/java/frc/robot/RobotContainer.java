// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmedSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.nav.NavSubsystem;

public abstract class RobotContainer extends ArmedSubsystem {

  /* Be sure to register all subsystems after they are created */
  protected DriveSubsystem drive;
  protected IntakeSubsystem intake;
  protected LauncherSubsystem launcher;
  protected NavSubsystem nav;


  protected RobotContainer() {
    super();
  }

  protected void registerSubsystems() {
    this.addChild(drive.getName(), drive);
    this.addChild(intake.getName(), intake);
    this.addChild(launcher.getName(), launcher);
    this.addChild(nav.getName(), nav);
  }

}
