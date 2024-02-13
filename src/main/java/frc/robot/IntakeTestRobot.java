// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.intake.RollerIntake;

public class IntakeTestRobot extends PracticeRobot {

  
  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public IntakeTestRobot() {
    super();
  }

  
    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
      intake.arm();
    }
        @Override
    public void teleopExit() {
      intake.disarm();
    }

  @Override
  public void teleopPeriodic() {
    // This method will be called once per scheduler run
    RunIntakeByJoystick();
    

  }
  private void RunIntakeByJoystick() {
    /* read gamepad and map inputs to robot functions */
    if (gamepad2.getXButton()) {
      intake.load();
    } else if (gamepad2.getYButton()) {
      intake.unload();
    } else if (gamepad2.getAButton()) {
      intake.stop();
    }
  }

  public void driveSpeedControl(double fwd, double strafe, double rotate){
    // Test robot has no drive system  
  }
}
