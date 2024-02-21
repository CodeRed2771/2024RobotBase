// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.HID.Gamepad;
import frc.robot.subsystems.drive.ExampleSwerveDriveTrain;
import frc.robot.subsystems.intake.DummyIntake;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RollerIntake;
import frc.robot.subsystems.launcher.DummyLauncher;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.launcher.RollerLauncher;
import frc.robot.subsystems.launcher.RollerLauncher.LauncherSpeeds;
import frc.robot.subsystems.nav.PracticeRobotNav;
import frc.robot.subsystems.auto.AutoBaseClass;

public class PracticeRobot extends DefaultRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";

  private AutoBaseClass mAutoProgram = null;

  /* Be sure to register all subsystems after they are created */
  public ExampleSwerveDriveTrain drive; // Changed from protected to public for autos
  public IntakeSubsystem intake; // Changed from protected to public for autos
  public RollerLauncher launcher; // Changed from protected to public for autos
  public PracticeRobotNav nav; // Changed from protected to public for autos

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

    /* Set all of the subsystems */
    nav = new PracticeRobotNav();
    drive = new ExampleSwerveDriveTrain(wiring);
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

    drive.updateOdometry(new Rotation2d(nav.getAngle()));
  }

  @Override
  public void autonomousInit() {
    
  }
  @Override
  public void autonomousPeriodic() {
    //blah
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
    driveAuxJoystick(gamepad1);
    SpeedDriveByJoystick(gamepad1);
    runLauncher(gamepad2);
  }



  @Override
  public void restoreRobotToDefaultState() {
    nav.reset();
    drive.reset(); // sets encoders based on absolute encoder positions
  }

  protected void driveAuxJoystick(Gamepad gp){
    // if(gp.getDPadLeft()) fieldCentricDriveMode(true);
    // if(gp.getDPadRight()) fieldCentricDriveMode(false);

    if(gp.getDPadDown()) driveSpeedGain = 0.25;
    if(gp.getDPadUp()) driveSpeedGain = 0.7;

    if(gp.getXButton()) nav.zeroYaw();
  }

  @Override
  public double getAngle(){return nav.getAngle();}

  protected double driveSpeedGain = 0.7;
  protected double rotateSpeedGain = 0.9;
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
      launcher.prime(LauncherSpeeds.SUBWOOFER.get());
    } else if(gp.getAButton()) {
      launcher.setSpeedBias(0);
      launcher.prime(LauncherSpeeds.SPEAKER.get());
    } else if(gp.getBButton()) {
      launcher.setSpeedBias(.15);
      launcher.prime(LauncherSpeeds.AMP.get());
    } else if(gp.getYButton()) {
      launcher.prime(0);
    }

    if (gp.getDPadRight() && !launcher.isLoaded()){
      launcher.load(.45);
    }
    else if (gp.getDPadUp()){
      launcher.load(.25);
    }
    else if (gp.getDPadLeft()) {
      launcher.unload();
    } else if(gp.getDPadDown() || gp.getDPadUp()) {
      launcher.stopLoader();
    }

    if (launcher.isLoaded() && !launcher.isFiring()&& !launcher.isUnloading()) {
      launcher.stopLoader();
    }

    if (gp.getRightTriggerAxis() > .5) {
      launcher.fire();
    }
  }

}
