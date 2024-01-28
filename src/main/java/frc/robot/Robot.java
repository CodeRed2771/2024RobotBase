// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.*;

import frc.robot.libs.HID.Gamepad;
import frc.robot.subsystems.drive.DriveTrain;
import frc.robot.subsystems.nav.NavXGyro;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
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
  //XboxController gamepad1;
  private RobotContainer myRobot;

  NavXGyro robotGyro;
  DriveTrain driveTrain;
  DriveAuto driveAuto;

  private Gamepad gamepad1;
  private Gamepad gamepad2;

  public enum RobotType{
    Dummy,
    IntakeTest,
    DriveTest,
    None
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Calibration.loadSwerveCalibration();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    robotGyro = NavXGyro.getInstance();
    driveTrain = DriveTrain.getInstance();
    driveAuto = DriveAuto.getInstance();

    robotGyro.init();
    driveTrain.init();
    // driveAuto.init();
    //gamepad1 = new XboxController(0);
    // SmartDashboard.putNumber("Mod A ABS", moduleA.)

    /* Replace this with the robot selection from pin strapping */
    var botType = RobotType.IntakeTest;

    switch (botType) {
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

    // driveAuto.tick();
    SmartDashboard.updateValues();
    driveTrain.smartDashboardOutputABSRotations();
    driveTrain.showTurnEncodersOnDash();

    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
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
    robotGyro.reset();
        
    driveTrain.stopDriveAndTurnMotors();
    driveTrain.allowTurnEncoderReset();
    driveTrain.resetTurnEncoders();
    driveTrain.setAllTurnOrientation(0, false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
      if (gamepad1.getStartButton()) {
          robotGyro.reset();
          
          driveTrain.allowTurnEncoderReset();
          driveTrain.resetTurnEncoders(); // sets encoders based on absolute encoder positions

          driveTrain.setAllTurnOrientation(0, false);
      }
      driveTrain.fieldCentricDrive(-gamepad1.getLeftY(), gamepad1.getLeftX(), gamepad1.getRightX());
    /* read gamepad and map inputs to robot functions*/
    if(gamepad2.getXButton()) {
      myRobot.intake.load();
    } else if(gamepad2.getYButton()){
      myRobot.intake.unload();
    } else {
      myRobot.intake.stop();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    Calibration.initializeSmartDashboard(); 
    myRobot.disarm();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    if (Calibration.shouldCalibrateSwerve()) {
      double[] pos = driveTrain.getAllAbsoluteTurnOrientations();
      Calibration.saveSwerveCalibration(pos[0], pos[1], pos[2], pos[3]);
  }

  // see if we want to reset the calibration to whatever is in the program
  // basically setting "Delete Swerve Calibration" to true will trigger
  // this, which deletes the calibration file.
  Calibration.checkIfShouldDeleteCalibration();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
        myRobot.arm();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
