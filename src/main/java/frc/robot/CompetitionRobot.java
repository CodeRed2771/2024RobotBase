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
    wiring.put("FL turn", 2);
    wiring.put("FL drive", 1);
    wiring.put("BR turn", 8);
    wiring.put("BR drive", 5);
    wiring.put("BL turn", 6);
    wiring.put("BL drive", 7);
    wiring.put("FR turn", 4);
    wiring.put("FR drive", 3);

    wiring.put("upper launcher",  20);
    wiring.put("lower launcher",  21);
    wiring.put("launcher loader",  23);
    wiring.put("load sensor", 4); // analog 4 (on the Navx MXP)rev 
    wiring.put("aim",  22);
    wiring.put("climber", 25);
    wiring.put("intakeMotorId", 16);

    // Analog Input
    wiring.put("FL turn enc", 1);
    wiring.put("BR turn enc", 0);
    wiring.put("BL turn enc", 2);
    wiring.put("FR turn enc", 3);

    // DIO Port
    wiring.put("aim encoder",  0);

    //PWM wiring
    wiring.put("launcher led", 0);
    wiring.put("nav led", 1);

    /* Tuning/calibration parameters that are robot specific go here */
    // Drive
    calibration.put("wheel base",23.5);

    /* Only adjust scale calibration after verifying the wheel alignment is good and accurate */
    double wheel_avg_error = 234.0 / 257.0; // Actual travel / Reported travel
    double drive_fwd_bias = 0.0; // Pull to right / distance travelled
    double drive_strafe_bias = 0.0; // pull to front / distance strafed
    calibration.put("FL wheel scale error", wheel_avg_error * (1 + drive_fwd_bias) * (1 - drive_strafe_bias) );
    calibration.put("BR wheel scale error", wheel_avg_error * (1 - drive_fwd_bias) * (1 + drive_strafe_bias) );
    calibration.put("BL wheel scale error", wheel_avg_error * (1 + drive_fwd_bias) * (1 + drive_strafe_bias) );
    calibration.put("FR wheel scale error", wheel_avg_error * (1 - drive_fwd_bias) * (1 - drive_strafe_bias) );

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
    nav = new PracticeRobotNav(wiring,calibration,drive);
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
