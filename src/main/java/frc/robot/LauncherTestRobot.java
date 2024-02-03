// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drive.DummyDrive;
import frc.robot.subsystems.intake.DummyIntake;
import frc.robot.subsystems.launcher.RollerLauncher;
import frc.robot.subsystems.nav.DummyNav;

public class LauncherTestRobot extends RobotContainer {

  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public LauncherTestRobot() {
    super();

    /* Set all of the subsystems */
    drive = new DummyDrive();
    intake = new DummyIntake();
    launcher = new RollerLauncher(3,1);
    nav = new DummyNav();
    registerSubsystems();
  }

  /* On program start, initialize any device settings or internal states of the subsystem. */
  @Override
  public void doArm() {
    launcher.arm();
  }

  @Override
  public void doDisarm() {
    launcher.disarm();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
