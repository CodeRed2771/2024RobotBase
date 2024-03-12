// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.TuneablePIDControllerGains;
import frc.robot.libs.HID.Gamepad;
import frc.robot.subsystems.drive.ExampleSwerveDriveTrain;
import frc.robot.subsystems.intake.DummyIntake;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.RollerLauncher;
import frc.robot.subsystems.launcher.RollerLauncher.LauncherSpeeds;
import frc.robot.subsystems.nav.PracticeRobotNav;
import frc.robot.subsystems.auto.AutoBaseClass;
import frc.robot.subsystems.auto.AutoCalibration;
import frc.robot.subsystems.auto.AutoDoNothing;
import frc.robot.subsystems.auto.AutoSpeaker2;

public class CrescendoBot extends DefaultRobot {

  // SendableChooser<String> autoChooser;
  SendableChooser<String> positionChooser;
  String autoSelected;
  private static final String kDefaultAuto = "Default";
  private final String kAutoShoot2 = "Auto Shoot 2 (CENTER)";
  private final String kAutoCalibration = "Auto Shoot 2";
  private final String autoDoNothing = "Auto Do Nothing";
  private static final String kCustomAuto = "My Auto";
  private static final double kMetersToInches = 100.0/2.54;

  private AutoBaseClass mAutoProgram = null;

  /* Be sure to register all subsystems after they are created */
  public ExampleSwerveDriveTrain drive; // Changed from protected to public for autos
  public IntakeSubsystem intake; // Changed from protected to public for autos
  public RollerLauncher launcher; // Changed from protected to public for autos
  public PracticeRobotNav nav; // Changed from protected to public for autos

  protected double driveSpeedGain = 1.0;
  protected double rotateSpeedGain = 0.9;
  
  protected double headingCmd;
  protected PIDController headingController = new PIDController(0,0,0);
  protected TuneablePIDControllerGains headingGains = new TuneablePIDControllerGains("Hdg", headingController);

  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public CrescendoBot() {
    super();

    headingGains.setP(5.0);
    headingGains.setI(0.0);
    headingGains.setD(0.0);

  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("Calibration Auto", kAutoCalibration);
    m_chooser.addOption("Auto Shoot 2 (CENTER)", kAutoShoot2);
    SmartDashboard.putData("Auto choices", m_chooser);
    setupAutoChoices();
  }

  /* Always update certian parts of the robot, like telemetry */
  @Override
  public void robotPeriodic(){
    super.robotPeriodic();
  }

  @Override
  public void autonomousInit() {
    String selectedPos = positionChooser.getSelected();
    SmartDashboard.putString("Position Chooser Selected", selectedPos);
    char robotPosition = selectedPos.toCharArray()[0];
    System.out.println("Robot position: " + robotPosition);

    intake.arm();
    launcher.arm();
    drive.arm();
    nav.reset();

    restoreRobotToDefaultState();

    // autoSelected = (String) autoChooser.getSelected();
    // SmartDashboard.putString("Auto Selected: ", autoSelected);

    // mAutoProgram = new AutoDoNothing();
    // mAutoProgram.start();

    mAutoProgram = new AutoDoNothing();
    mAutoProgram.start();

    switch (autoSelected) {
      case kAutoShoot2:
          mAutoProgram = new AutoSpeaker2(this);
          mAutoProgram.start();
          break;
      case kAutoCalibration:
          mAutoProgram = new AutoCalibration(this);
          mAutoProgram.start();
          break;
    }
  }
  @Override
  public void autonomousPeriodic() {
    if (mAutoProgram.isRunning())
      mAutoProgram.periodic();
  }

  private void setupAutoChoices() {
    // Position Chooser
    positionChooser = new SendableChooser<String>();
    positionChooser.addOption("Left", "L");
    positionChooser.setDefaultOption("Center", "C");
    positionChooser.addOption("Right", "R");
    SmartDashboard.putData("Position", positionChooser);

    // autoChooser = new SendableChooser<String>();
    // // autoChooser.addOption(autoCalibrator, autoCalibrator);
    // //autoChooser.addOption(autoWheelAlign, autoWheelAlign);
    // // autoChooser.addOption(autoAlign, autoAlign);
    // //autoChooser.addOption(ballPickUp, ballPickUp);
    // autoChooser.addOption(AutoCommunity, AutoCommunity);
    // autoChooser.addOption(AutoCPlace1, AutoCPlace1);
    // // autoChooser.addOption(AutoCPlace2Wings, AutoCPlace2Wings);
    // autoChooser.addOption(AutoCP2RailRider, AutoCP2RailRider);
    // autoChooser.setDefaultOption(AutoCP1CB, AutoCP1CB);
    // autoChooser.addOption(AutoC_CB, AutoC_CB);
    // // autoChooser.addOption(AutoCPlace3VROOOM, AutoCPlace3VROOOM);
    
 
    // SmartDashboard.putData("Auto Chose:", autoChooser);

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
  protected void postTuneParams(){
    super.postTuneParams();
    headingGains.postTuneParams();
  }

  @Override
  protected void handleTuneParams(){
    super.postTuneParams();
    headingGains.handleTuneParams();
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
