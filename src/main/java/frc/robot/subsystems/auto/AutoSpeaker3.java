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
  aims & shoots 1 note,
  Drives forward and picks up another note (Leaving the starting area in the process)
  Drives back and aligns with subwoofer
  aims & shoots the other note
 */
import frc.robot.subsystems.launcher.RollerLauncherCompetition.LauncherPresets;


public class AutoSpeaker3 extends AutoBaseClass {

  private char position = 'C';
  private Optional<Alliance> alliance;
  final double DRIVE_TOLERANCE = 0.75;

  public AutoSpeaker3(CrescendoBot robot, char position) {
    super(robot);
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
              if(position == 'A')
                myRobot.drive.driveFixedPositionOffsetInches(48,-57);
              else
                myRobot.drive.driveFixedPositionOffsetInches(48, 57);
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
              if(position == 'A')
                myRobot.drive.driveFixedPositionOffsetInches(48,57);
              else
                myRobot.drive.driveFixedPositionOffsetInches(48, -57);
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
              myRobot.drive.driveFixedPositionOffsetInches(60, 0);
              myRobot.launcher.load(0.45);
              setTimerAndAdvanceStep(4000);
              break;
            case 11:
              if(myRobot.launcher.isLoaded())
                myRobot.launcher.stopLoader();
              if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE))
                advanceStep();
              break;
            case 12:
              myRobot.drive.driveFixedPositionOffsetInches(60, 0);
              myRobot.launcher.aim(LauncherPresets.SUBWOOFER);
              setTimerAndAdvanceStep(-4000);
              break;
            case 13:
            if(myRobot.launcher.isLoaded())
              myRobot.launcher.stopLoader();
            if(myRobot.drive.atFixedPosition(DRIVE_TOLERANCE))
              advanceStep();
            break;
            case 14:
            myRobot.launcher.fire();
            setTimerAndAdvanceStep(750);
            break;
            case 15:
              myRobot.launcher.stop();
              stop();
              break;
        }
      }
    }
}
