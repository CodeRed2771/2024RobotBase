// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drive.ExampleSwerveDriveTrain;
import frc.robot.subsystems.intake.DummyIntake;
import frc.robot.subsystems.launcher.RollerLauncherCompetition;
import frc.robot.subsystems.nav.PracticeRobotNav;

public class ElegantPolyBot extends CrescendoBot {

  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public ElegantPolyBot() {
    super();

    /*
     * Define all of the wiring for the robot in a common spot here and then pass it
     * around
     */
    wiring.put("A turn", 2);
    wiring.put("A drive", 1);
    wiring.put("B turn", 8);
    wiring.put("B drive", 5);
    wiring.put("C turn", 6);
    wiring.put("C drive", 7);
    wiring.put("D turn", 4);
    wiring.put("D drive", 3);

    wiring.put("A turn enc", 1);
    wiring.put("B turn enc", 2);
    wiring.put("C turn enc", 0);
    wiring.put("D turn enc", 3);

    wiring.put("upper launcher",  20);
    wiring.put("lower launcher",  21);
    wiring.put("launcher loader",  23);
    wiring.put("load sensor", 4); // analog 4 (on the Navx MXP)
    wiring.put("aim",  0);

    wiring.put("intakeMotorId", 16);

    //PWM wiring
    wiring.put("launcher led", 0);

    // Calibrated by measuring circumference in cm and calculating radius
    calibration.put("A wheel radius", 32.0 / 2.54 / (2* Math.PI));
    calibration.put("B wheel radius", 31.4 / 2.54 / (2* Math.PI));
    calibration.put("C wheel radius", 30.7 / 2.54 / (2* Math.PI));
    calibration.put("D wheel radius", 31.7 / 2.54 / (2* Math.PI));

    /* Set all of the subsystems */
    drive = new ExampleSwerveDriveTrain(wiring, calibration);
    nav = new PracticeRobotNav(calibration,drive);
    intake = new DummyIntake();
    launcher = new RollerLauncherCompetition(wiring, calibration);
  }

}
