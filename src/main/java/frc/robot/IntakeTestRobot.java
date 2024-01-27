// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drive.DummyDrive;
import frc.robot.subsystems.intake.RollerIntake;
import frc.robot.subsystems.launcher.DummyLauncher;
import frc.robot.subsystems.nav.DummyNav;

public class IntakeTestRobot extends RobotContainer {

  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public IntakeTestRobot() {
    super("Fake Robot");
    createRobot();
  }

  /** Creates a new RobotContainer. */
  public IntakeTestRobot(String name) {
    super(name);
    createRobot();
  }

  private void createRobot() {
    /* Set all of the subsystems */
    drive = new DummyDrive("No Wheels");
    intake = new RollerIntake();
    launcher = new DummyLauncher("Airball");
    nav = new DummyNav("Lost Boys");
    registerSubsystems();
  }

  /* On program start, initialize any device settings or internal states of the subsystem. */
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
