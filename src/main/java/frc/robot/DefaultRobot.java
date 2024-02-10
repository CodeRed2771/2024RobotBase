// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.libs.HID.Gamepad;

public class DefaultRobot extends TimedRobot {

  protected String m_autoSelected;
  protected final SendableChooser<String> m_chooser = new SendableChooser<>();

  protected Map<String,Integer> wiring;
  protected Gamepad gamepad1 = new Gamepad(0);
  protected Gamepad gamepad2 = new Gamepad(1);
  protected Gamepad gamepad3 = new Gamepad(2);

  public DefaultRobot() {
    super();

    wiring = new HashMap<>();
  }

  public void restoreRobotToDefaultState(){}


  /* By default just pass commands to the drive system */
  public void driveSpeedControl(double fwd, double strafe, double rotate){
    // By default do nothing for driving
  }

  public double getAngle(){return 0.0;}
  public void driveSpeedControlFieldCentric(double fwd, double strafe, double rotate) {
    /*
     * Rotate the drive command into field centric orientation by reversing out the
     * orientation of the robot
     */
    Translation2d command = new Translation2d(fwd, strafe);
    command = command.rotateBy(Rotation2d.fromDegrees(-this.getAngle()));

    driveSpeedControl(command.getX(), command.getY(), rotate);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // NOTE: If there are no commands registered, this calls the subsystems().periodic() function

    SmartDashboard.updateValues();

  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

  }


  public void SpeedDriveByJotstick(){
      double fwd = MathUtil.applyDeadband(-gamepad1.getLeftY(),0.05);
      double strafe = MathUtil.applyDeadband(-gamepad1.getLeftX(),0.05);
      double rotate = MathUtil.applyDeadband(-gamepad1.getRightX(),0.05);
      driveSpeedControl(fwd, strafe, rotate);

  }
}
