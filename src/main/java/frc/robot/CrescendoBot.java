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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.TuneablePIDControllerGains;
import frc.robot.libs.HID.Gamepad;
import frc.robot.subsystems.drive.ExampleSwerveDriveTrain;
import frc.robot.subsystems.intake.DummyIntake;
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
import frc.robot.subsystems.auto.AutoShootAndLeave;
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
  
  protected double autoAimAngle = 0;
  protected double autoAimPower = 0;

  protected boolean bHeadingHold = true;
  protected double headingCmd;
  protected PIDController headingController = new PIDController(0,0,0);
  protected TuneablePIDControllerGains headingGains = new TuneablePIDControllerGains("Hdg", headingController);

  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public CrescendoBot() {
    super();

    headingGains.setP(3.0);
    headingGains.setI(0.0);
    headingGains.setD(0.01);

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
    autoSelected = m_chooser.getSelected();
    setHeadingHoldAngle(getAngle());

    Optional<Alliance> alliance = DriverStation.getAlliance();

    if(alliance.get() == Alliance.Blue)
      Crescendo.useBlueTargets();
    else
      Crescendo.useRedTargets();

    switch(selectedPos){
      case "L":
        nav.reset(Crescendo.getPose3d(Crescendo.PointsOfInterest.SUBWOOFER_LEFT).toPose2d());
        break;
      case "R":
        nav.reset(Crescendo.getPose3d(Crescendo.PointsOfInterest.SUBWOOFER_RIGHT).toPose2d());
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
        mAutoProgram = new AutoDoNothing();
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
    // nav.reset();
    climber.reset(); // added 3/12/24 - not tested yet
    launcher.reset();

    climb_time = 0.0;
    climbing = false;

    restoreRobotToDefaultState();
    fieldCentricDriveMode(true);

    setHeadingHoldAngle(getAngle());
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

    Translation2d driveCmd = getJoystickDriveCommand(gp);

    double rotate = calculatedProfileYawCmd(-gp.getRightX());
    driveCmd = calculateProfiledDriveCommand(driveCmd);

    if(gp.getStartButtonPressed())
    {
      nav.zeroYaw();
      setHeadingHoldAngle(getAngle());
    }

    if(gp.getXButton())
    {
      rotate += MathUtil.clamp(getAngle()/120,-0.5,0.5);
    }

    if(bAutoAimEnabled) {
      rotate+=nav.yawRotationNudge();
    } else if(noteNudge) {
      rotate += nav.noteYawNudge();
    }

    if(! MathUtil.isNear(0.0,Math.abs(rotate),0.01))
    {
      setHeadingHoldAngle(getAngle());
    }

    if(bHeadingHold){
      rotate += calculateHeadingHoldCommand();
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

    if(gamepad1.getStartButtonPressed())
    {
      nav.zeroYaw();
    }
  }

  protected void setHeadingHoldAngle(double angle){
    headingCmd = angle;
  }

  protected double calculateHeadingHoldCommand(){
    double rotationError = MathUtil.inputModulus(headingCmd - nav.getAngle(),-180.0, 180.0) / 360.0;
    return headingController.calculate(rotationError);
  }



  @Override
  protected void postTuneParams(){
    super.postTuneParams();
    headingGains.postTuneParams();
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
    headingGains.handleTuneParams();
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
  }

  @Override
  public void restoreRobotToDefaultState() {
    drive.reset(); // sets encoders based on absolute encoder positions
    // nav.reset();

    setHeadingHoldAngle(getAngle());

    super.restoreRobotToDefaultState();
  }
  boolean bAutoAimEnabled = false;
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

    
    // if(gp.getAButton()) {
    //   bAutoAimEnabled = true;
    // } else {
    //   bAutoAimEnabled = false;
    // }

    // if(false && gp.getYButton()) {
    //   noteNudge = true;
    // } else {
    //   noteNudge = false;
    // }
  
  }

  public void autoAimLauncherAtSpeaker(){
    Translation3d target = Crescendo.getPose3d(PointsOfInterest.SPEAKER).getTranslation();
    Translation2d aimer = new Translation2d(10,0);
    Translation2d cur_pos = nav.getPoseInField().getTranslation().plus(aimer.rotateBy(new Rotation2d(nav.getAngle())));
    double range = target.toTranslation2d().minus(cur_pos).getNorm();
    double height = target.getZ() - 6.0; // offset for pivot point of launcher

    double angle = Math.toDegrees(Math.atan2(height,range));
  
    autoAimAngle = angle + 0.02 * range - 6.0;

    autoAimPower = 2450 + 5 * range;

    SmartDashboard.putNumber("Auto Aim Angle", autoAimAngle);
    SmartDashboard.putNumber("Auto Aim Power", autoAimPower);
    SmartDashboard.putNumber("Auto Range", range);
    SmartDashboard.putNumber("Auto Angle", angle);
    SmartDashboard.putNumber("Auto Target X", target.getX());
    SmartDashboard.putNumber("Auto Target Y", target.getY());
    SmartDashboard.putNumber("Auto Target Z", target.getZ());

    launcher.aim(autoAimAngle);
    launcher.prime(autoAimPower);
  }

  @Override
  public double getAngle(){return nav.getAngle();}

  /* By default just pass commands to the drive system */
  @Override
  public void driveSpeedControl(Translation2d driveCmd, double rotate) {
    driveCmd = driveCmd.times(driveSpeedGain);
    if((driveCmd.getNorm()+Math.abs(rotate)) < 0.05)
      drive.driveHoldWheels();
    else
      drive.driveSpeedControl(driveCmd.getX(), driveCmd.getY(), rotate*rotateSpeedGain,getPeriod());
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
      last_LauncherCommand = LauncherPresets.SAFE_ZONE;
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
      last_LauncherCommand= LauncherPresets.OFF;
    }

    if(bAutoAimEnabled && ( last_LauncherCommand == LauncherPresets.SUBWOOFER || last_LauncherCommand == LauncherPresets.SAFE_ZONE)){
      if(nav.isNavValid()){
        autoAimLauncherAtSpeaker();
      }
      else if(last_LauncherCommand != null) {
        launcher.aim(last_LauncherCommand);
      }
    }

  }

}
