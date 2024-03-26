// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auto;

import java.util.Optional;
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
              myRobot.drive.driveFixedPositionOffsetInches(60,0);
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
              myRobot.drive.driveFixedPositionOffsetInches(-60,0);
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
              myRobot.launcher.prime(LauncherSpeeds.SUBWOOFER);
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
              myRobot.launcher.prime(LauncherSpeeds.OFF);
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
                myRobot.drive.driveFixedRotatePosition(35);
              }
              else {
                myRobot.drive.driveFixedRotatePosition(-35);
              }
              setTimerAndAdvanceStep(2000);
              break;
            case 7:
              if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE))
                advanceStep();
              break;
            case 8:
              myRobot.launcher.load(0.45);
              myRobot.drive.driveFixedPositionOffsetInches(60,0);
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
              myRobot.drive.driveFixedPositionOffsetInches(-60,0);
              setTimerAndAdvanceStep(4000);
              break;
            case 11:
              if(myRobot.launcher.isLoaded())
                myRobot.launcher.stopLoader();
              if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE)) {
                advanceStep();
              }
              break;
            case 12:
              if((position == 'A' && alliance.get()==Alliance.Blue) || (position == 'S' && alliance.get()==Alliance.Blue)) {
                myRobot.drive.driveFixedRotatePosition(-35);
              }
              else {
                myRobot.drive.driveFixedRotatePosition(35);
              }
              setTimerAndAdvanceStep(2000);
              break;
            case 13:
              if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE))
                advanceStep();
              break;
            case 14:
              myRobot.launcher.prime(LauncherSpeeds.SUBWOOFER);
              myRobot.drive.driveFixedPositionOffsetInches(-20,0);
              myRobot.launcher.load(0.45);
              setTimerAndAdvanceStep(2000);
              break;
            case 15:
              if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE)) {
                advanceStep();
              }
              break;
            case 16:
              myRobot.launcher.fire();
              setTimerAndAdvanceStep(1000);
              break;
            case 17:
              break;
            case 18:
              myRobot.drive.driveFixedPositionOffsetInches(80, 0);
              setTimerAndAdvanceStep(5000);
              break;
            case 19:
              if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE))
                advanceStep();
              break;
            case 20:
              myRobot.launcher.stop();
              stop();
              break;
            }
        }
      }
    }
}
