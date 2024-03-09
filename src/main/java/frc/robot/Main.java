// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {

  public enum RobotType{
    Dummy,
    IntakeTest,
    DriveTest,
    TankBot,
    None
  }

  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    /* Replace this with the robot selection from pin strapping */
    var botType = RobotType.DriveTest;

    switch (botType) {
      case DriveTest:
        RobotBase.startRobot(PracticeRobot::new);
        break;
      case IntakeTest:
        RobotBase.startRobot(IntakeTestRobot::new);
        break;
      case TankBot:
        RobotBase.startRobot(TankBot::new);
        break;
      default:
        RobotBase.startRobot(DefaultRobot::new);
    }
  }
}
