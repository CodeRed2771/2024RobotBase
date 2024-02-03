// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.libs.HID.Gamepad;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 * 
 * 
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static double cmb;

  private RobotContainer myRobot;

  private Gamepad gamepad1;
  private Gamepad gamepad2;

  public enum RobotType {
    Dummy,
    IntakeTest,
    DriveTest,
    None
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

    // gamepad1 = new XboxController(0);
    // SmartDashboard.putNumber("Mod A ABS", moduleA.)

    /* Replace this with the robot selection from pin strapping */
    var botType = RobotType.DriveTest;

    switch (botType) {
      case DriveTest:
        myRobot = new PracticeRobot();
        break;
      case IntakeTest:
        myRobot = new IntakeTestRobot();
        break;
      default:
        myRobot = new DummyRobot();
    }

    gamepad1 = new Gamepad(0);
    gamepad2 = new Gamepad(1);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // NOTE: If there are no commands registered, this calls the
    // subsystems().periodic() function

    SmartDashboard.updateValues();

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    myRobot.arm();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    myRobot.arm();
    myRobot.restoreRobotToDefaultState();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (gamepad1.getStartButton()) {
      myRobot.restoreRobotToDefaultState();
    }

    double fwd = MathUtil.applyDeadband(-gamepad1.getLeftY(), 0.02);
    double strafe = MathUtil.applyDeadband(gamepad1.getLeftX(), 0.02);
    double rotate = MathUtil.applyDeadband(gamepad1.getRightX(), 0.02);
    myRobot.driveSpeedControlFieldCentric(fwd, strafe, rotate);

    /* read gamepad and map inputs to robot functions */
  }

  static double speed = 0;
  static double bias = 0;

  public void runLauncher() {
    if (gamepad2.getXButtonPressed()) {
      speed = Math.min(1.0,speed + .05);
    }
    if (gamepad2.getYButtonPressed()) {
      speed = Math.max(0.0,speed - .05);
    }
    if (gamepad2.getAButtonPressed()) {
      bias = Math.min(.05,bias + .01);
    }
    if (gamepad2.getBButtonPressed()) {
      bias = Math.max(-.05,bias - .01);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    myRobot.disarm();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    myRobot.arm();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
