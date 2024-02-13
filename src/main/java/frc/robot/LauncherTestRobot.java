// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.launcher.RollerLauncher;

public class LauncherTestRobot extends PracticeRobot {

  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public LauncherTestRobot() {
    super();
    launcher = new RollerLauncher(wiring);
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
    runLauncher(gamepad1);
  }

}
