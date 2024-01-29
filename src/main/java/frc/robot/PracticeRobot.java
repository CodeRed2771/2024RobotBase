// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drive.PracticeDriveTrain;
import frc.robot.subsystems.intake.DummyIntake;
import frc.robot.subsystems.launcher.DummyLauncher;
import frc.robot.subsystems.nav.PracticeRobotNav;

public class PracticeRobot extends RobotContainer {

  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public PracticeRobot() {
    super();

    /* Set all of the subsystems */
    drive = PracticeDriveTrain.getInstance();
    intake = new DummyIntake();
    launcher = new DummyLauncher();
    nav = new PracticeRobotNav();
    registerSubsystems();
  }

  /*
   * On program start, initialize any device settings or internal states of the
   * subsystem.
   */
  @Override
  public void doArm() {
    intake.arm();
    launcher.arm();
    drive.arm();
  }

  @Override
  public void doDisarm() {
    intake.disarm();
    launcher.disarm();
    drive.disarm();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}