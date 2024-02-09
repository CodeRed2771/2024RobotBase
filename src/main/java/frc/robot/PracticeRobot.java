// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.drive.PracticeDriveTrain;
import frc.robot.subsystems.intake.RollerIntake;
import frc.robot.subsystems.launcher.DummyLauncher;
import frc.robot.subsystems.launcher.RollerLauncher;
import frc.robot.subsystems.nav.PracticeRobotNav;

public class PracticeRobot extends RobotContainer {

  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public PracticeRobot() {
    super();

    /* Define all of the wiring for the robot in a common spot here and then pass it around */
    wiring.put("A turn", 2);
    wiring.put("A drive", 1);
    wiring.put("B turn", 8);
    wiring.put("B drive", 7);
    wiring.put("C turn",  6);
    wiring.put("C drive", 5);
    wiring.put("D turn",  4);
    wiring.put("D drive", 3);

    wiring.put("A turn enc",1 );
    wiring.put("B turn enc",2 );
    wiring.put("C turn enc",0 );
    wiring.put("D turn enc",3 );
  
    wiring.put("NavX",  SPI.Port.kMXP.value);

    wiring.put("upper launcher",  20);
    wiring.put("lower launcher",  21);
    wiring.put("launcher loader",  23);
    wiring.put("load sensor", 4); // analog 4 (on the Navx MXP)
    wiring.put("aim",  0);

    wiring.put("intakeMotorId", 16);

    /* Set all of the subsystems */
    drive = new PracticeDriveTrain(wiring);
    intake = new RollerIntake(wiring);
    launcher = new RollerLauncher(wiring);
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
    nav.reset();
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

  @Override
  public void restoreRobotToDefaultState() {
    nav.reset();
    drive.reset(); // sets encoders based on absolute encoder positions
    ((PracticeDriveTrain) drive).setAllTurnOrientation(0, false);
  }

}
