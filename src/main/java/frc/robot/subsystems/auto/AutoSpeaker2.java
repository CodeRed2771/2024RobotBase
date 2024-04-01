// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auto;

import java.util.Optional;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CrescendoBot;
/*
  This auto (that starts in the center position):
  Primes & shoots 1 note,
  Drives forward and picks up another note (Leaving the starting area in the process)
  Drives back and aligns with subwoofer
  Primes & shoots the other note
 */
import frc.robot.subsystems.launcher.RollerLauncherCompetition.LauncherPresets;


public class AutoSpeaker2 extends AutoBaseClass {

  CrescendoBot myRobot;
  private char position = 'C';
  private Optional<Alliance> alliance;
  final double DRIVE_TOLERANCE = 0.75;
  Translation2d noteDrive;

  public AutoSpeaker2(CrescendoBot robot, char position) {
    super();
    myRobot = robot;
    this.position = position;
  }
  public void start() {
		super.start();
    alliance = DriverStation.getAlliance();
	}  
  public void stop() {
    super.stop();
  }
  public void periodic() {
    if (isRunning()) {
        SmartDashboard.putNumber("Auto Step", getCurrentStep());
        if(position == 'C') {
        switch (getCurrentStep()) {
            case 0:
              myRobot.launcher.aim(LauncherPresets.SUBWOOFER);
              setTimerAndAdvanceStep(1500);
              break;
            case 1:
              break;
            case 2:
              myRobot.launcher.fire();
              setTimerAndAdvanceStep(1000);
              break;
            case 3:
              break;
            case 4:
              myRobot.launcher.stopFireDelay();
              myRobot.launcher.aim(LauncherPresets.OFF);
              noteDrive = new Translation2d(60,0);
              noteDrive = noteDrive.rotateBy(Rotation2d.fromDegrees(-1.5*MathUtil.clamp(myRobot.nav.getBearingToNote(),-30,30)));
              myRobot.drive.driveFixedPositionOffsetInches(noteDrive.getX(),noteDrive.getY());
              myRobot.launcher.load(0.45);
              setTimerAndAdvanceStep(4000);
              break;
            case 5:
              if(myRobot.launcher.isLoaded())
                myRobot.launcher.stopLoader();
              if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE)) {
                advanceStep();
              }
              break;
            case 6:
              myRobot.launcher.aim(LauncherPresets.SUBWOOFER);
              myRobot.drive.driveFixedPositionOffsetInches(-noteDrive.getX(),-noteDrive.getY());
              setTimerAndAdvanceStep(4000);
              break;
            case 7:
              if(myRobot.launcher.isLoaded())
                myRobot.launcher.stopLoader();
              if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE)) {
                advanceStep();
              }
              break;
            case 8:
              myRobot.launcher.fire();
              setTimerAndAdvanceStep(1000);
              break;
            case 9:
              break;
            case 10:
              myRobot.drive.driveFixedPositionOffsetInches(80, 0);
              setTimerAndAdvanceStep(5000);
              break;
            case 11:
              if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE))
                advanceStep();
              break;
            case 12:
              myRobot.launcher.stop();
              stop();
              break;
            }
        }
        else {
        switch (getCurrentStep()) {
            case 0:
              myRobot.launcher.aim(LauncherPresets.SUBWOOFER);
              setTimerAndAdvanceStep(1500);
              break;
            case 1:
              break;
            case 2:
              myRobot.launcher.fire();
              setTimerAndAdvanceStep(500);
              break;
            case 3:
              break;
            case 4:
              myRobot.launcher.stopFireDelay();
              myRobot.launcher.aim(LauncherPresets.OFF);
              myRobot.drive.driveFixedPositionOffsetInches(20,0);
              myRobot.launcher.load(0.45);
              setTimerAndAdvanceStep(2000);
              break;
            case 5:
              if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE)) {
                advanceStep();
              }
              break;
            case 6:
              if((position == 'A' && alliance.get()==Alliance.Blue) || (position == 'S' && alliance.get()==Alliance.Blue)) {
                myRobot.drive.driveFixedRotatePosition(60);
              }
              else {
                myRobot.drive.driveFixedRotatePosition(-60);
              }
              setTimerAndAdvanceStep(2000);
              break;
            case 7:
              if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE))
                advanceStep();
              break;
            case 8:
              myRobot.launcher.load(0.45);
              noteDrive = new Translation2d(65,0);
              noteDrive = noteDrive.rotateBy(Rotation2d.fromDegrees(-1.5*MathUtil.clamp(myRobot.nav.getBearingToNote(),-25,25)));
              myRobot.drive.driveFixedPositionOffsetInches(noteDrive.getX(),noteDrive.getY());
              setTimerAndAdvanceStep(4000);
              break;
            case 9:
              if(myRobot.launcher.isLoaded())
                myRobot.launcher.stopLoader();
              if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE)) {
                advanceStep();
              }
              break;
            case 10:
              myRobot.drive.driveFixedPositionOffsetInches(-noteDrive.getX(),-noteDrive.getY());
              setTimerAndAdvanceStep(4000);
              break;
            case 11:
              if(myRobot.launcher.isLoaded())
                myRobot.launcher.stopLoader();
              if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE*2)) {
                advanceStep();
              }
              break;
            case 12:
              if((position == 'A' && alliance.get()==Alliance.Blue) || (position == 'S' && alliance.get()==Alliance.Blue)) {
                myRobot.drive.driveFixedRotatePosition(-60);
              }
              else {
                myRobot.drive.driveFixedRotatePosition(60);
              }
              setTimerAndAdvanceStep(2000);
              break;
            case 13:
              myRobot.autoAimLauncherAtSpeaker();
              if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE))
                advanceStep();
              break;
            case 14:
              myRobot.autoAimLauncherAtSpeaker();
              // myRobot.launcher.aim(LauncherPresets.SUBWOOFER);
              // myRobot.drive.driveFixedPositionOffsetInches(-20,0);
              // myRobot.launcher.load(0.45);
              //setTimerAndAdvanceStep(1000);
              advanceStep();
              break;
            case 15:
              advanceStep();
              // if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE)) {
              //   advanceStep();
              // }
              break;
            case 16:
              myRobot.launcher.fire();
              setTimerAndAdvanceStep(1000);
              break;
            case 17:
              break;
            case 18:

              if((position == 'A' && alliance.get()==Alliance.Blue) || (position == 'S' && alliance.get()==Alliance.Blue))
                myRobot.drive.driveFixedPositionOffsetInches(30, -100);
              else
                myRobot.drive.driveFixedPositionOffsetInches(30, 100);

              setTimerAndAdvanceStep(5000);
              break;
            case 19:
              if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE))
                advanceStep();
              break;
            default:
              myRobot.launcher.stop();
              stop();
              break;
            }
        }
      }
    }
}
