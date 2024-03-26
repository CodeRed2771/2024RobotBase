// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.ExampleSwerveDriveTrain;
import frc.robot.subsystems.intake.DummyIntake;
import frc.robot.subsystems.launcher.RollerLauncherCompetition;
import frc.robot.subsystems.nav.PracticeRobotNav;

public class CompetitionRobot extends CrescendoBot {

  /** Creates a new RobotContainer. */
  @SuppressWarnings("this-escape")
  public CompetitionRobot() {
    super();

    /*
     * Define all of the wiring for the robot in a common spot here and then pass it
     * around
     */
    // CANBUS Device ID
    wiring.put("A turn", 2);
    wiring.put("A drive", 1);
    wiring.put("B turn", 8);
    wiring.put("B drive", 5);
    wiring.put("C turn", 6);
    wiring.put("C drive", 7);
    wiring.put("D turn", 4);
    wiring.put("D drive", 3);

    wiring.put("upper launcher",  20);
    wiring.put("lower launcher",  21);
    wiring.put("launcher loader",  23);
    wiring.put("load sensor", 4); // analog 4 (on the Navx MXP)rev 
    wiring.put("aim",  22);
    wiring.put("climber", 25);
    wiring.put("intakeMotorId", 16);

    // Analog Input
    wiring.put("A turn enc", 1);
    wiring.put("B turn enc", 0);
    wiring.put("C turn enc", 2);
    wiring.put("D turn enc", 3);

    // DIO Port
    wiring.put("aim encoder",  0);

    //PWM wiring
    wiring.put("launcher led", 0);

    /* Tuning/calibration parameters that are robot specific go here */
    // Drive
    calibration.put("wheel base",23.5);

    // Launcher
    calibration.put("upper launcher direction", 1.0);
    calibration.put("aim bias",-10.0);
    
    //Climber
    calibration.put("climber P", 2.0);//was .07
    calibration.put("climber I", 0.0);
    calibration.put("climber D", 0.0);
    calibration.put("climber Izone", 0.0);
    calibration.put("climber velocity", 25.0);
    calibration.put("climber acceleration", 10.0);
    
    // Navigation Camera Install
    calibration.put("limelight X",-6.0);
    calibration.put("limelight Y",-12.0);
    calibration.put("limelight Z",15.0);
    calibration.put("limelight roll",0.0);
    calibration.put("limelight pitch",-40.0);
    calibration.put("limelight yaw",180.0);
    calibration.put("limelight present", 1.0);

    calibration.put("note threshold", 1100.0);

    /* Set all of the subsystems */
    drive = new ExampleSwerveDriveTrain(wiring, calibration);
    nav = new PracticeRobotNav(calibration,drive);
    intake = new DummyIntake();
    launcher = new RollerLauncherCompetition(wiring, calibration);
    climber = new Climber(wiring, calibration);
  }

  @Override
  public void teleopPeriodic(){
    SwerveModulePosition[] startPos = drive.getOdomotry();

    super.teleopPeriodic();

    SwerveModulePosition[] stopPos = drive.getOdomotry();
    double[] deltas = {0,0,0,0};
    for(int i = 0;i<4;i++){
      deltas[i] = stopPos[i].angle.getRadians() - startPos[i].angle.getRadians();
    }
    SmartDashboard.putNumberArray("deltas ",deltas);
    
  }
}
