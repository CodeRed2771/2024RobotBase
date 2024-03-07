// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.HID.Gamepad;
import frc.robot.subsystems.drive.TankDriveSubsystem;

public class TankBot extends DefaultRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";

  /* Be sure to register all subsystems after they are created */
  protected TankDriveSubsystem drive;
 
  protected double driveSpeedGain = 1.0;
  protected double rotateSpeedGain = 0.9;
  
  protected double kHeadingAccelLim = 1.0 / 2.0 ; // Max cmd / Time to achieve Cmd
  protected SlewRateLimiter hdgAccelSlew = new SlewRateLimiter(kHeadingAccelLim);


  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public TankBot() {
    super();

    /*
     * Define all of the wiring for the robot in a common spot here and then pass it
     * around
     */
    wiring.put("A drive", 1);
    wiring.put("B drive", 5);

    /* Set all of the subsystems */
    drive = new TankDriveSubsystem(wiring, calibration);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }


  /*
   * On program start, initialize any device settings or internal states of the
   * subsystem.
   */
  @Override
  public void teleopInit() {
    bDriveFieldCentric = false;
    hdgAccelSlew .reset(0);
    drive.arm();
  }

  @Override
  public void teleopExit() {
    drive.disarm();
  }

  @Override
  public void teleopPeriodic() {
    // This method will be called once per scheduler run
    adjustDriveSpeed(gamepad1);
    SpeedDriveByJoystick(gamepad1);
  }

  @Override
  protected void SpeedDriveByJoystick(Gamepad gp) {

    Translation2d driveCmd = getJoystickDriveCommand(gp);

    double rotate = calculatedProfileYawCmd(-gp.getRightX());
    driveCmd = calculateProfiledDriveCommand(driveCmd);

    if (bDriveFieldCentric) {
      driveSpeedControlFieldCentric( driveCmd, rotate);
    } else {

      driveSpeedControl( driveCmd, rotate);
    }
  }

  @Override
  public void disabledInit(){
    postTuneParams();
  }

  @Override
  public void disabledPeriodic(){
    handleTuneParams();
  }

  /* Compute the profiled yaw command given a rotation Command of +/- 1.0 */
  protected double calculatedProfileYawCmd(double rotateCmd){
    // Integrate rotate to move heading command
    rotateCmd = MathUtil.applyDeadband(rotateCmd, 0.05);
    return hdgAccelSlew.calculate(rotateCmd);
  }


  private void postTuneParams(){
    SmartDashboard.putNumber("Hdg R Accel Lim", kHeadingAccelLim);

    SmartDashboard.putNumber("Drive Accel Lim P", kDrivePosAccelLim);
    SmartDashboard.putNumber("Drive Accel Lim N", kDriveNegAccelLim);
  }
  private void handleTuneParams(){

    double val;
    boolean changed = false;
    val = SmartDashboard.getNumber("Hdg R Accel Lim", kHeadingAccelLim);
    if (val != kHeadingAccelLim){
      kHeadingAccelLim = val;
      hdgAccelSlew = new SlewRateLimiter(kHeadingAccelLim);
    }
    val = SmartDashboard.getNumber("Drive Accel Lim P", kDrivePosAccelLim);
    if (val != kDrivePosAccelLim){
      kDrivePosAccelLim = val;
      changed = true;
    }
    val = SmartDashboard.getNumber("Drive Accel Lim N", kDriveNegAccelLim);
    if (val != kDriveNegAccelLim){
      kDriveNegAccelLim = val;
      changed = true;
    }
    if(changed)
      driveAccelSlew = new SlewRateLimiter(kDrivePosAccelLim,kDriveNegAccelLim,0.0);
  }

  protected void adjustDriveSpeed(Gamepad gp){
    if(gp.getDPadUp()) fieldCentricDriveMode(true);
    if(gp.getDPadDown()) fieldCentricDriveMode(false);

    if(gp.getRightBumper()) {
      driveSpeedGain = 0.25;
      rotateSpeedGain = 0.25;
    } else if (gp.getLeftBumper()) {
      driveSpeedGain = 0.6;
      rotateSpeedGain = 0.6;
    } else {
      driveSpeedGain = 1.0;
      rotateSpeedGain = 1.0;
    }

  }

  /* By default just pass commands to the drive system */
  @Override
  public void driveSpeedControl(Translation2d driveCmd, double rotate) {
    driveCmd = driveCmd.times(driveSpeedGain);
    drive.driveSpeedControl(driveCmd.getX(), driveCmd.getY(), rotate*rotateSpeedGain);
  }

}
