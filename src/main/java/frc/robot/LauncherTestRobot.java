// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.drive.DummyDrive;
import frc.robot.subsystems.drive.PracticeDriveTrain;
import frc.robot.subsystems.intake.DummyIntake;
import frc.robot.subsystems.intake.RollerIntake;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.launcher.RollerLauncher;
import frc.robot.subsystems.nav.DummyNav;
import frc.robot.subsystems.nav.PracticeRobotNav;

public class LauncherTestRobot extends PracticeRobot {

  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public LauncherTestRobot() {
    super();
  }

    /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (gamepad1.getStartButton()) {
      restoreRobotToDefaultState();
    }

    double fwd = MathUtil.applyDeadband(-gamepad1.getLeftY(), 0.02);
    double strafe = MathUtil.applyDeadband(gamepad1.getLeftX(), 0.02);
    double rotate = MathUtil.applyDeadband(gamepad1.getRightX(), 0.02);
    driveSpeedControlFieldCentric(fwd*0.5, strafe*0.5, rotate*0.5);

    /* read gamepad and map inputs to robot functions */
   // runLauncher();

    if (gamepad1.getLeftBumper()) {
      launcher.prime(.5);
    } else if(gamepad1.getLeftBumperReleased()) {
      launcher.stopLoader();
    }  else {
      launcher.prime(0);
    }

    if (gamepad1.getAButton() && !launcher.isLoaded()){
      // myRobot.intake.load();
      launcher.load(.75);
    }
    else if (gamepad1.getYButton()) {
      // myRobot.intake.unload();
      launcher.unload();
    } else if(gamepad1.getYButtonReleased()) {
      launcher.stopLoader();
    }
    else if (gamepad1.getXButton()){
      // myRobot.intake.stop();  
      launcher.stopLoader();
    }

    if (launcher.isLoaded() && !launcher.isFiring()&& !launcher.isUnloading()) {
      // myRobot.intake.stop();  
      launcher.stopLoader();
    }

    if (gamepad1.getRightTriggerAxis() > .5) {
      // future - check if primed first - leaving that out for testing
      launcher.fire();
    }
  }

  private double speed = 0;
  private double bias = 0;

  public void runLauncher() {
    launcher.prime(gamepad1.getRightTriggerAxis());
  }

}
