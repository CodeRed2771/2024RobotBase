// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.TuneablePIDControllerGains;
import frc.robot.libs.HID.Gamepad;
import frc.robot.subsystems.drive.ExampleSwerveDriveTrain;
import frc.robot.subsystems.intake.DummyIntake;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.RollerLauncherCompetition;
import frc.robot.subsystems.launcher.RollerLauncher.LauncherSpeeds;
import frc.robot.subsystems.nav.PracticeRobotNav;

public class CompetitionRobot extends DefaultRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final double kMetersToInches = 100.0/2.54;

  /* Be sure to register all subsystems after they are created */
  public ExampleSwerveDriveTrain drive;
  public IntakeSubsystem intake;
  public RollerLauncherCompetition launcher;
  public PracticeRobotNav nav;

  protected double driveSpeedGain = 1.0;
  protected double rotateSpeedGain = 0.9;
  
  protected double headingCmd;
  protected PIDController headingController = new PIDController(0,0,0);
  protected TuneablePIDControllerGains headingGains = new TuneablePIDControllerGains("Hdg", headingController);

  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public CompetitionRobot() {
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
    wiring.put("B turn enc", 0);
    wiring.put("C turn enc", 2);
    wiring.put("D turn enc", 3);

    wiring.put("upper launcher",  20);
    wiring.put("lower launcher",  21);
    wiring.put("launcher loader",  23);
    wiring.put("load sensor", 4); // analog 4 (on the Navx MXP)rev 
    wiring.put("aim",  22);

    // DIO Port
    wiring.put("aim encoder",  0);

    wiring.put("intakeMotorId", 16);

    //PWM wiring
    wiring.put("launcher led", 0);

    calibration.put("upper launcher direction", 1.0);
    calibration.put("note threshold", 1100.0);

    headingGains.setP(5.0);
    headingGains.setI(0.0);
    headingGains.setD(0.0);

    /* Set all of the subsystems */
    drive = new ExampleSwerveDriveTrain(wiring, calibration);
    nav = new PracticeRobotNav(drive);
    intake = new DummyIntake();
    launcher = new RollerLauncherCompetition(wiring, calibration);

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

  /* Always update certian parts of the robot, like telemetry */
  @Override
  public void robotPeriodic(){
    super.robotPeriodic();
  }

  /*
   * On program start, initialize any device settings or internal states of the
   * subsystem.
   */
  @Override
  public void teleopInit() {
    super.teleopInit();

    intake.arm();
    launcher.arm();
    drive.arm();
    nav.reset();

    restoreRobotToDefaultState();
    fieldCentricDriveMode(true);

  }

  @Override
  public void teleopExit() {
    super.teleopExit();
    intake.disarm();
    launcher.disarm();
    drive.disarm();
  }

  @Override
  public void teleopPeriodic() {
    super.teleopPeriodic();
    // This method will be called once per scheduler run
    adjustDriveSpeed(gamepad1);
    SpeedDriveByJoystick(gamepad1);
    runLauncher(gamepad2);
  }

  @Override
  protected void SpeedDriveByJoystick(Gamepad gp) {

    Translation2d driveCmd = getJoystickDriveCommand(gp);

    double rotate = calculatedProfileYawCmd(-gp.getRightX());
    driveCmd = calculateProfiledDriveCommand(driveCmd);

    if(ampNudge) {
      rotate+=nav.yawRotationNudge();
    } else if(noteNudge) {
      rotate += nav.noteYawNudge();
    }

    if (bDriveFieldCentric) {
      driveSpeedControlFieldCentric( driveCmd, rotate);
    } else {

      driveSpeedControl( driveCmd, rotate);
    }
  }

  @Override
  public void disabledInit(){
    super.disabledInit();

    intake.disarm();
    launcher.disarm();
    drive.disarm();

    postTuneParams();
  }

  @Override
  public void disabledPeriodic(){
    super.disabledPeriodic();
    handleTuneParams();
  }

  protected void resetLimitedHeadingControl(){
    headingCmd = getAngle();
    hdgAccelSlew .reset(0);
  }

  protected double calculateRotationCommand(double heading){
    double rotationError = MathUtil.inputModulus(heading - nav.getAngle(),-180.0, 180.0) / 360.0;
    return headingController.calculate(rotationError);
  }

  protected void speedDriveByJoystickHeading(Gamepad gp) {

    Translation2d driveCmd = getJoystickDriveCommand(gp);
    driveCmd = calculateProfiledDriveCommand(driveCmd);

    double rotate = MathUtil.applyDeadband(-gp.getRightX(), 0.05);
    double hdg = calculatedProfileYawCmd(rotate);
    rotate = calculateRotationCommand(hdg);

    driveSpeedControlFieldCentric(driveCmd, rotate);
  }

  @Override
  public void restoreRobotToDefaultState() {
    drive.reset(); // sets encoders based on absolute encoder positions
    nav.reset();

    resetLimitedHeadingControl();

    super.restoreRobotToDefaultState();
  }
  boolean ampNudge = false;
  boolean noteNudge = false;
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

    if(gp.getXButton()) nav.zeroYaw();

    if(gp.getAButton()) {
      ampNudge = true;
      resetLimitedHeadingControl();
    } else {
      ampNudge = false;
    }

    if(gp.getYButton()) {
      noteNudge = true;
    } else {
      noteNudge = false;
    }
  }

  @Override
  public double getAngle(){return nav.getAngle();}

  /* By default just pass commands to the drive system */
  @Override
  public void driveSpeedControl(Translation2d driveCmd, double rotate) {
    driveCmd = driveCmd.times(driveSpeedGain);
    drive.driveSpeedControl(driveCmd.getX(), driveCmd.getY(), rotate*rotateSpeedGain,getPeriod());
  }

  private double speed = 0;
  private double bias = 0;

  public void runLauncher(Gamepad gp) {
    if (gp.getXButton()) {
      launcher.setSpeedBias(0);
      launcher.prime(LauncherSpeeds.SUBWOOFER);
    } else if(gp.getAButton()) {
      launcher.setSpeedBias(0);
      launcher.prime(LauncherSpeeds.SAFE_ZONE);
    } else if(gp.getBButton()) {
      launcher.setSpeedBias(.15);
      launcher.prime(LauncherSpeeds.AMP);
    } else if(gp.getYButton()) {
      launcher.prime(LauncherSpeeds.OFF);
    }

    if (gp.getDPadRight() && !launcher.isLoaded()){
      launcher.load(.45);
    }
    else if (gp.getDPadUp()){
      launcher.load(.25);
    }
    else if (gp.getDPadLeft()) {
      launcher.unload();
    } else if(gp.getDPadDown()) {
      launcher.stopLoader();
    } else if (launcher.isLoaded() && !launcher.isFiring()&& !launcher.isUnloading()) {
      launcher.stopLoader();
    }

    if (gp.getRightTriggerAxis() > .5) {
      launcher.fire();
    }
  }

}
