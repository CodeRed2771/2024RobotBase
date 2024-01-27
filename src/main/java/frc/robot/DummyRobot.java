// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drive.DummyDrive;
import frc.robot.subsystems.intake.DummyIntake;
import frc.robot.subsystems.launcher.DummyLauncher;
import frc.robot.subsystems.nav.DummyNav;

public class DummyRobot extends RobotContainer {

  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public DummyRobot() {
    super();

    /* Set all of the subsystems */
    drive = new DummyDrive();
    intake = new DummyIntake();
    launcher = new DummyLauncher();
    nav = new DummyNav();
    registerSubsystems();
  }

  /*
   * On program start, initialize any device settings or internal states of the
   * subsystem.
   */
  public void arm() {
    intake.arm();
  }

  public void disarm() {
    intake.disarm();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
