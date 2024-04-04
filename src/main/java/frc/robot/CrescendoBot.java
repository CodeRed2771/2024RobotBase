// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.TuneablePIDControllerGains;
import frc.robot.libs.HID.Gamepad;
import frc.robot.subsystems.drive.ExampleSwerveDriveTrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.RollerLauncherCompetition;
import frc.robot.subsystems.launcher.RollerLauncherCompetition.LauncherPresets;
import frc.robot.subsystems.nav.Crescendo;
import frc.robot.subsystems.nav.PracticeRobotNav;
import frc.robot.subsystems.nav.Crescendo.PointsOfInterest;
import frc.robot.subsystems.auto.AutoBaseClass;
import frc.robot.subsystems.auto.AutoCalibration;
import frc.robot.subsystems.auto.AutoDoNothing;
import frc.robot.subsystems.auto.AutoShootAndLeave;
import frc.robot.subsystems.auto.AutoSpeaker2;
import frc.robot.subsystems.climber.Climber;

public class CrescendoBot extends DefaultRobot {

  // SendableChooser<String> autoChooser;
  SendableChooser<String> positionChooser;
  String autoSelected;
  private static final String autoShoot2 = "Auto Shoot 2";
  private static final String autoShootAndLeave = "Auto Shoot and Leave";
  private static final String autoDoNothing = "Do Nothing";
  private static final String kCalibration = "Cal Auto";

  private static final double kMetersToInches = 100.0/2.54;

  private AutoBaseClass mAutoProgram = null;

  /* Be sure to register all subsystems after they are created */
  public ExampleSwerveDriveTrain drive; // Changed from protected to public for autos
  public IntakeSubsystem intake; // Changed from protected to public for autos
  public RollerLauncherCompetition launcher; // Changed from protected to public for autos
  public PracticeRobotNav nav; // Changed from protected to public for autos
  public Climber climber; 

  protected double driveSpeedGain = 1.0;
  protected double rotateSpeedGain = 0.4;
  
  protected boolean bHeadingHold = true;
  protected boolean bGoingUnder = false;
  protected double headingHoldAngle;
  protected PIDController headingController = new PIDController(0,0,0);
  private double headingHoldLimit = 0.25;
  protected TuneablePIDControllerGains headingHoldGains = new TuneablePIDControllerGains("Hdg", headingController);

  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public CrescendoBot() {
    super();

    headingHoldGains.setP(5.0 / 360.0);
    headingHoldGains.setI(0.0 / 360.0);
    headingHoldGains.setIz(0.0 / 360.0);
    headingHoldGains.setD(0.01 / 360.0);

  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    setupAutoChoices();
  }

  /* Always update certian parts of the robot, like telemetry */
  @Override
  public void robotPeriodic(){
    super.robotPeriodic();

    postTelemetry();

    monitorSpeedLowerLauncher();
  }

  public void initializeFieldData(){
    nav.initializeFieldData();

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if(alliance.get() == Alliance.Blue)
      Crescendo.useBlueTargets();
    else
      Crescendo.useRedTargets();
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
    initializeFieldData();

    nav.disableCamera();
    restoreRobotToDefaultState();
    autoSelected = m_chooser.getSelected();

    switch(selectedPos){
      case "A":
        nav.reset(Crescendo.getPose3d(Crescendo.PointsOfInterest.SUBWOOFER_AMP).toPose2d());
        break;
      case "S":
        nav.reset(Crescendo.getPose3d(Crescendo.PointsOfInterest.SUBWOOFER_SOURCE).toPose2d());
        break;
      default:
        nav.reset(Crescendo.getPose3d(Crescendo.PointsOfInterest.SUBWOOFER_CENTER).toPose2d());
        break;
    }
  
    SmartDashboard.putString("Auto Selected", autoSelected);
  
    switch(autoSelected){
      case kCalibration:
        mAutoProgram = new AutoCalibration(this);
        break;
      case autoShoot2:
        mAutoProgram = new AutoSpeaker2(this, robotPosition);
        break;
      case autoShootAndLeave:
        mAutoProgram = new AutoShootAndLeave(this, robotPosition);
        break;
      default:
        mAutoProgram = new AutoDoNothing(this);
        break;
    }
    mAutoProgram.start();
  }

  @Override
  public void autonomousPeriodic() {
    if (mAutoProgram.isRunning())
      mAutoProgram.periodic();
  }

  private void setupAutoChoices() {
    // Position Chooser
    positionChooser = new SendableChooser<String>();
    positionChooser.addOption("AMP SIDE", "A");
    positionChooser.setDefaultOption("CENTER", "C");
    positionChooser.addOption("SOURCE SIDE", "S");

    SmartDashboard.putData("Position", positionChooser);

    m_chooser.setDefaultOption("Default Auto", autoDoNothing);
    m_chooser.addOption(kCalibration, kCalibration);
    m_chooser.addOption(autoShoot2, autoShoot2);
    m_chooser.addOption(autoShootAndLeave, autoShootAndLeave);

    SmartDashboard.putData("Auto choices", m_chooser);
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
    climber.reset(); // added 3/12/24 - not tested yet
    launcher.reset();
    initializeFieldData();


    climb_time = 0.0;
    climbing = false;

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
    runClimber(gamepad2);
  }

  private boolean climbing = false;
  private double climb_time = 0;
  private double kP_climb = 1.0;
    protected void runClimber(Gamepad gp) {
      double speed = kP_climb * MathUtil.applyDeadband(gp.getLeftY(), 0.05);
      // climber.lift(speed, true);
      if(gp.getRightBumper()) {
        climbing = true;
        climber.lift(speed, true);
        climber.reset();
      } else {
        
        if(Math.abs(speed) > 0.1) {
          climbing = true;
          climb_time += getPeriod();
        }
        else 
        {
          climbing = false;
        }
        climber.lift(speed, false);
      }
    }


  @Override
  protected void SpeedDriveByJoystick(Gamepad gp) {

    /* Apply driver commands */
    Translation2d driveCmd = getJoystickDriveCommand(gp);
    driveCmd = calculateProfiledDriveCommand(driveCmd);
    double rotate = calculatedProfileYawCmd(-gp.getRightX());

    /* Apply speed augmentation settings */
    driveCmd = driveCmd.times(driveSpeedGain);
    rotate *= rotateSpeedGain;

    /* Apply any commanded automatic aim operations */
    if(gp.getXButton()) {
      rotate += computeHeadingTurnControl(0.0);
    }
    if(gp.getBButton()) {
      if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        rotate += computeHeadingTurnControl(90.0);
      else
        rotate += computeHeadingTurnControl(-90.0);
    }
    if(gp.getAButton() && nav.isNavValid()) {
      rotate += computeTurnController(nav.getBearingToTarget(Crescendo.getPose3d(PointsOfInterest.SPEAKER)) + 180);
    }
    if(gp.getYButton() && !nav.isTrackingNote()) {
      rotate += computeTurnController(nav.getBearingToNote());
    }

    /* Grab the last heading reading as the rotate command is released */
    if (!MathUtil.isNear(0.0, Math.abs(rotate), 0.01)) {
      headingHoldAngle = getAngle();
    } else {
      rotate += computeHeadingTurnControl(headingHoldAngle);
    }

    /* Apply the command to the drive system */
    if (bDriveFieldCentric) {
      driveSpeedControlFieldCentric( driveCmd, rotate);
    } else {

      driveSpeedControl( driveCmd, rotate);
    }

    if(gp.getStartButtonPressed()) {
      nav.zeroYaw();
      headingHoldAngle = getAngle();
    }
  }

  @Override
  public void disabledInit(){
    super.disabledInit();

    intake.disarm();
    launcher.disarm();
    drive.disarm();
    nav.enableCamera();


    last_LauncherCommand = LauncherPresets.OFF;
    postTuneParams();
  }

  @Override
  public void disabledPeriodic(){
    super.disabledPeriodic();
      handleTuneParams();

    if(gamepad1.getStartButtonPressed()) {
      nav.zeroYaw();
    }
  }

  protected double computeTurnController(double bearing){
    bearing = MathUtil.inputModulus(bearing,-180.0, 180.0);
    double rotationError = MathUtil.applyDeadband(bearing, 0.1,180.0);
    return MathUtil.clamp(headingController.calculate(rotationError),-headingHoldLimit,headingHoldLimit);
  }

  public double getBearingToHeading(double goal){
    return  MathUtil.inputModulus(goal - nav.getAngle(),-180.0, 180.0);
  }

  @Override
  protected void postTuneParams(){
    super.postTuneParams();
    headingHoldGains.postTuneParams();
    launcher.postTuneParameters();
    
    SmartDashboard.putBoolean("Heading hold", bHeadingHold);
    SmartDashboard.putBoolean("Camera Enable", nav.isCameraEnabed());
    SmartDashboard.putNumber("AMP angle", aim);
    SmartDashboard.putNumber("AMP power", speed);
    SmartDashboard.putNumber("AMP bias", bias);

  }

  @Override
  protected void handleTuneParams(){
    super.handleTuneParams();
    headingHoldGains.handleTuneParams();
    launcher.handleTuneParameters();

    bHeadingHold = SmartDashboard.getBoolean("Heading hold", bHeadingHold);
    nav.setCameraEnable(SmartDashboard.getBoolean("Camera Enable", nav.isCameraEnabed()));

    aim = SmartDashboard.getNumber("AMP angle", aim);
    speed = SmartDashboard.getNumber("AMP power", speed);
    bias = SmartDashboard.getNumber("AMP bias", bias);

  }

  @Override
  public void postTelemetry(){
    nav.postTelemetry();
    SmartDashboard.putNumber("Heading Hold Angle", headingHoldAngle);
  }

  @Override
  public void restoreRobotToDefaultState() {
    drive.reset(); // sets encoders based on absolute encoder positions

    headingHoldAngle = getAngle();

    super.restoreRobotToDefaultState();
  }
  boolean bAutoAimEnabled = false;
  protected void adjustDriveSpeed(Gamepad gp){
    if(gp.getDPadUp()) fieldCentricDriveMode(true);
    if(gp.getDPadDown()) fieldCentricDriveMode(false);

    if(gp.getRightBumper()) {
      driveSpeedGain = 0.25;
      rotateSpeedGain = 0.10;
    } else if (gp.getLeftBumper()) {
      driveSpeedGain = 0.6;
      rotateSpeedGain = 0.25;
    } else {
      driveSpeedGain = 1.0;
      rotateSpeedGain = 0.4;
    }

    if(gp.getRightTriggerAxis() > .5){
      bGoingUnder = true;
    } else {
      bGoingUnder = false;
    }
  }

  public void autoPrimeLauncherToSpeaker(){
    Transform3d aimer = new Transform3d(10,0,6,new Rotation3d()); // offset for pivot point of launcher
    Pose3d cur_pos = new Pose3d(nav.getPoseInField());
    cur_pos = cur_pos.transformBy(aimer);

    Transform3d target = Crescendo.getPose3d(PointsOfInterest.SPEAKER).minus(cur_pos);

    SmartDashboard.putNumber("Auto Target X", target.getX());
    SmartDashboard.putNumber("Auto Target Y", target.getY());
    SmartDashboard.putNumber("Auto Target Z", target.getZ());

    double range = target.getTranslation().toTranslation2d().getNorm();
    double height = target.getZ(); // offset for pivot point of launcher
    launcher.autoAimLauncher(range, height);
  }

  @Override
  public double getAngle(){return nav.getAngle();}

  /* By default just pass commands to the drive system */
  @Override
  public void driveSpeedControl(Translation2d driveCmd, double rotate) {
    if((driveCmd.getNorm()+Math.abs(rotate)) < 0.05)
      drive.driveHoldWheels();
    else
      drive.driveSpeedControl(driveCmd.getX(), driveCmd.getY(), rotate,getPeriod());
  }

  private double speed = LauncherPresets.AMP.getSpeed();
  private double aim =  LauncherPresets.AMP.getAngle();
  private double bias = LauncherPresets.AMP.getBias();

  private double last_slow_time = 0.0;
  private double lower_launcher_delay = 1.0;
  private LauncherPresets last_LauncherCommand = LauncherPresets.OFF;
  private void monitorSpeedLowerLauncher(){
    if(nav.getVelInRobot().getNorm() < 48.0)
      last_slow_time =  Timer.getFPGATimestamp();
  }
  private void holdoffShouldLowerLauncher(double time){
      last_slow_time =  Timer.getFPGATimestamp() + time - lower_launcher_delay;

  }
  private boolean shouldLowerLauncher()
  {
    return (Timer.getFPGATimestamp() - last_slow_time > lower_launcher_delay) && (launcher.getAngle() > LauncherPresets.OFF.getAngle());
  }

  private boolean bFiring = false;
  public void runLauncher(Gamepad gp) {

    // if(shouldLowerLauncher())
    //   launcher.aim(LauncherPresets.OFF.getAngle());
    if( climbing ){
      last_LauncherCommand = LauncherPresets.CLIMB;
    }
    
    /* Target shooting the speaker from preset locations */
    if (gp.getXButton()) {
      bAutoAimEnabled = true;
      last_LauncherCommand = LauncherPresets.SAFE_ZONE;
    } else if(gp.getAButton()) {
      bAutoAimEnabled = true;
      last_LauncherCommand = LauncherPresets.SUBWOOFER;
    } else {
      bAutoAimEnabled = false;
    }

    if(gp.getBButton()) {
      last_LauncherCommand = LauncherPresets.AMP;
    }
    
    if(gp.getYButton()) {
      last_LauncherCommand = LauncherPresets.OFF;
    } 
    
    if (gp.getDPadRight() && !launcher.isLoaded()){
      last_LauncherCommand = LauncherPresets.PICKUP;
      launcher.load(.45);
    }
    else if (gp.getDPadUp()){
      launcher.load(.25);
    }
    else if (gp.getDPadLeft()) {
      last_LauncherCommand = LauncherPresets.PICKUP;
      launcher.unload();
    } else if(gp.getDPadDown()) {
      launcher.stopLoader();
    } else if (launcher.isLoaded() && !launcher.isFiring()&& !launcher.isUnloading()) {
      launcher.stopLoader();
    }

    if (gp.getRightTriggerAxis() > .5) {
      launcher.fire();
      bFiring = true;
    }

    if(bFiring && !launcher.isFiring()) {
      last_LauncherCommand = LauncherPresets.OFF;
    }
    if(last_LauncherCommand == LauncherPresets.OFF)
      bFiring = false;


    if(bGoingUnder) {
      launcher.prime(last_LauncherCommand.getSpeed());
      launcher.aim(LauncherPresets.OFF.getAngle());
    } else if(bAutoAimEnabled && ( last_LauncherCommand == LauncherPresets.SUBWOOFER || last_LauncherCommand == LauncherPresets.SAFE_ZONE)){
      if(nav.isNavValid()){
        autoPrimeLauncherToSpeaker();
      }
      else if(last_LauncherCommand != null) {
        launcher.aim(last_LauncherCommand);
      }
    } else {
      launcher.aim(last_LauncherCommand);
    }
  }

  /* +Heading is to left in NEU, +bearing is right in robot frame */
  public double computeHeadingTurnControl(double heading){
    return computeTurnController(getBearingToHeading(heading));
  }

}
