// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.ExampleSwerveDriveTrain;
import frc.robot.subsystems.intake.DummyIntake;
import frc.robot.subsystems.intake.RollerIntake;
import frc.robot.subsystems.launcher.DummyLauncher;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.nav.PracticeRobotNav;

public class PracticeRobot extends DefaultRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";

  /* Be sure to register all subsystems after they are created */
  protected ExampleSwerveDriveTrain drive;
  protected RollerIntake intake;
  protected LauncherSubsystem launcher;
  protected PracticeRobotNav nav;

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
    wiring.put("B drive", 7);
    wiring.put("C turn", 6);
    wiring.put("C drive", 5);
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
    intake = new RollerIntake(wiring);
    launcher = new DummyLauncher();

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
    SpeedDriveByJotstick();
    //RunIntakeByJoystick();
  }

  @Override
  public void restoreRobotToDefaultState() {
    nav.reset();
    drive.reset(); // sets encoders based on absolute encoder positions
  }

  private void RunIntakeByJoystick() {
    /* read gamepad and map inputs to robot functions */
    if (gamepad2.getXButton()) {
      intake.load();
    } else if (gamepad2.getYButton()) {
      intake.unload();
    } else if (gamepad2.getAButton()) {
      intake.stop();
    }
  }

  /* By default just pass commands to the drive system */
  @Override
  public void driveSpeedControl(double fwd, double strafe, double rotate) {
    drive.driveSpeedControl(fwd, strafe, rotate,getPeriod());
  }

}
