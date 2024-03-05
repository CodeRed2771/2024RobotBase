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
import frc.robot.libs.HID.Gamepad;
import frc.robot.subsystems.drive.ExampleSwerveDriveTrain;
import frc.robot.subsystems.intake.DummyIntake;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.RollerLauncher;
import frc.robot.subsystems.launcher.RollerLauncher.LauncherSpeeds;
import frc.robot.subsystems.nav.PracticeRobotNav;

public class PracticeRobot extends DefaultRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final double kMetersToInches = 100.0/2.54;

  /* Be sure to register all subsystems after they are created */
  protected ExampleSwerveDriveTrain drive;
  protected IntakeSubsystem intake;
  protected RollerLauncher launcher;
  protected PracticeRobotNav nav;

  protected double driveSpeedGain = 1.0;
  protected double rotateSpeedGain = 0.9;

  protected double kDrivePosAccelLim = 1.0 / 2.0 ; // Max cmd / Time to achieve Cmd
  protected double kDriveNegAccelLim = -1.0 / 0.25 ; // Max cmd / Time to achieve Cmd
  protected SlewRateLimiter driveAccelSlew = new SlewRateLimiter(kDrivePosAccelLim,kDriveNegAccelLim,0);
  
  protected double kHeadingRateLim = 180.0;
  protected double kHeadingAccelLim = 360.0 / 1.0 ; // Max cmd / Time to achieve Cmd
  protected SlewRateLimiter hdgAccelSlew = new SlewRateLimiter(kHeadingAccelLim);
  protected double headingCmd;
  protected PIDController headingController;

  protected double kHeadingP = 5.0;
  protected double kHeadingI = 0.0;
  protected double kHeadingD = 0.0;
  protected double kHeadingFF = 0.0;

  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public PracticeRobot() {
    super();
    wiring = new HashMap<>();

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

    headingController = new PIDController(kHeadingP,kHeadingI,kHeadingD);

    /* Set all of the subsystems */
    drive = new ExampleSwerveDriveTrain(wiring);
    nav = new PracticeRobotNav(drive);
    intake = new DummyIntake();
    launcher = new RollerLauncher(wiring);

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
    intake.arm();
    launcher.arm();
    drive.arm();
    nav.reset();

    restoreRobotToDefaultState();
    fieldCentricDriveMode(true);

  }

  @Override
  public void teleopExit() {
    intake.disarm();
    launcher.disarm();
    drive.disarm();
  }

  @Override
  public void teleopPeriodic() {
    // This method will be called once per scheduler run
    adjustDriveSpeed(gamepad1);
    SpeedDriveByJoystick(gamepad1);
    runLauncher(gamepad2);
  }

  @Override
  protected void SpeedDriveByJoystick(Gamepad gp) {
    double fwd = MathUtil.applyDeadband(-gp.getLeftY(), 0.05);
    double strafe = MathUtil.applyDeadband(-gp.getLeftX(), 0.05);
    double rotate = MathUtil.applyDeadband(-gp.getRightX(), 0.05);

    Translation2d driveCmd = new Translation2d(-gp.getLeftY(),-gp.getLeftX());
    driveCmd = calculateProfiledDriveCommand(driveCmd);
    fwd = driveCmd.getX();
    strafe = driveCmd.getY();

    if(ampNudge) {
      rotate+=nav.yawRotationNudge();
    }

    if (bDriveFieldCentric) {
      driveSpeedControlFieldCentric(fwd, strafe, rotate);
    } else {

      driveSpeedControl(fwd, strafe, rotate);
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

  protected void resetLimitedHeadingControl(){
    headingCmd = nav.getAngle();
    hdgAccelSlew .reset(0);
  }

  public double wrapDegreesTo180(double angle){
    double rotations = angle / 360.0;
    rotations -= (int) rotations;
    if (rotations >= 0.5) rotations -= 1.0;
    return rotations * 360.0;
  }

  /* Compute the profiled yaw command given a rotation Command of +/- 1.0 */
  protected double calculatedProfileYawCmd(double rotateCmd){
    double yawRate = kHeadingRateLim * getPeriod() * rotateCmd; 
    // Integrate rotate to move heading command
    yawRate = hdgAccelSlew.calculate(yawRate);
    headingCmd = headingCmd + yawRate;

    return headingCmd;
  }

  protected double calculateRotationCommand(double heading){
    double rotationError = wrapDegreesTo180(heading - nav.getAngle()) / 360.0;
    return headingController.calculate(rotationError);
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

  protected void speedDriveByJoystickHeading(Gamepad gp) {

    Translation2d driveCmd = new Translation2d(-gp.getLeftY(),-gp.getLeftX());
    driveCmd = calculateProfiledDriveCommand(driveCmd);

    double rotate = MathUtil.applyDeadband(-gp.getRightX(), 0.05);
    double hdg = calculatedProfileYawCmd(rotate);
    rotate = calculateRotationCommand(hdg);

    driveSpeedControlFieldCentric(driveCmd.getX(), driveCmd.getY(), rotate);
  }

  private void postTuneParams(){
    SmartDashboard.putNumber("Hdg R Lim", kHeadingRateLim);
    SmartDashboard.putNumber("Hdg R Accel Lim", kHeadingAccelLim);
    SmartDashboard.putNumber("Hdg P", kHeadingP);
    SmartDashboard.putNumber("Hdg I", kHeadingI);
    SmartDashboard.putNumber("Hdg D", kHeadingD);

    SmartDashboard.putNumber("Drive Accel Lim P", kDrivePosAccelLim);
    SmartDashboard.putNumber("Drive Accel Lim N", kDriveNegAccelLim);
  }
  private void handleTuneParams(){

    double val;
    boolean changed = false;
    val = SmartDashboard.getNumber("Hdg R Lim", kHeadingRateLim);
    if (val != kHeadingRateLim){
      kHeadingRateLim = val;
    }
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
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Hdg P", kHeadingP);
    double i = SmartDashboard.getNumber("Hdg I", kHeadingI);
    double d = SmartDashboard.getNumber("Hdg D", kHeadingD);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((p != kHeadingP)) {
      headingController.setP(p);
      kHeadingP = p;
    }
    if ((i != kHeadingI)) {
      headingController.setI(i);
      kHeadingI = i;
    }
    if ((d != kHeadingD)) {
      headingController.setD(d);
      kHeadingD = d;
    }
  }

  @Override
  public void restoreRobotToDefaultState() {
    drive.reset(); // sets encoders based on absolute encoder positions
    nav.reset();

    driveAccelSlew.reset(0);
    resetLimitedHeadingControl();
  }
  boolean ampNudge = false;
  protected void adjustDriveSpeed(Gamepad gp){
    if(gp.getDPadUp()) fieldCentricDriveMode(true);
    if(gp.getDPadDown()) fieldCentricDriveMode(false);

    if(gp.getRightBumper()) {
      driveSpeedGain = 0.25;
      rotateSpeedGain = 0.25;
    } else if (gp.getLeftBumper()) {
      driveSpeedGain = 0.7;
      rotateSpeedGain = 0.7;
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
  }

  @Override
  public double getAngle(){return nav.getAngle();}

  /* By default just pass commands to the drive system */
  @Override
  public void driveSpeedControl(double fwd, double strafe, double rotate) {
    drive.driveSpeedControl(fwd*driveSpeedGain, strafe*driveSpeedGain, rotate*rotateSpeedGain,getPeriod());
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
