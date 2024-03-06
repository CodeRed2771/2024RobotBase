// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

  protected double kDrivePosAccelLim = 1.0 / 2.0 ; // Max cmd / Time to achieve Cmd
  protected double kDriveNegAccelLim = -1.0 / 0.25 ; // Max cmd / Time to achieve Cmd
  protected SlewRateLimiter driveAccelSlew = new SlewRateLimiter(kDrivePosAccelLim,kDriveNegAccelLim,0);


  public DefaultRobot() {
    super();

    wiring = new HashMap<>();
  }

  public void restoreRobotToDefaultState(){
    driveAccelSlew.reset(0);
  }


  /* By default just pass commands to the drive system */
  public void driveSpeedControl(Translation2d driveCmd, double rotate){
    // By default do nothing for driving
  }

  public double getAngle(){return 0.0;}
  public void driveSpeedControlFieldCentric(Translation2d command, double rotate) {
    /*
     * Rotate the drive command into field centric orientation by reversing out the
     * orientation of the robot
     */

    driveSpeedControl(command.rotateBy(Rotation2d.fromDegrees(getAngle())), rotate);
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

  protected boolean bDriveFieldCentric = true;
  protected void fieldCentricDriveMode(boolean mode) {
      bDriveFieldCentric = mode;
  }

  protected Translation2d getJoystickDriveCommand(Gamepad gp) {
    return new Translation2d(-gp.getLeftY(),-gp.getLeftX());
  }

  protected Translation2d calculateProfiledDriveCommand(Translation2d command){
    double magnitude = command.getNorm();
    Rotation2d angle = command.getAngle();

    if(magnitude < 0.075) // Deadband out 0.05 rotationally
    {
      magnitude = 0.0;
    }

    magnitude = driveAccelSlew.calculate(magnitude);

    return new Translation2d(magnitude,angle);
  }

  protected void SpeedDriveByJoystick(Gamepad gp) {
    Translation2d driveCmd = getJoystickDriveCommand(gp);
    double rotate = MathUtil.applyDeadband(-gp.getRightX(), 0.05);
    if (bDriveFieldCentric) {
      driveSpeedControlFieldCentric(driveCmd, rotate);
    } else {
      driveSpeedControl(driveCmd, rotate);
    }
  }
}
