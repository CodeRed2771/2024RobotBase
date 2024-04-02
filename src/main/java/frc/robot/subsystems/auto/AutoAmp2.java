// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DefaultRobot;
import frc.robot.CrescendoBot;
/*
  This auto (that starts in the center position):
  Primes & shoots 1 note,
  Drives forward and picks up another note (Leaving the starting area in the process)
  Drives back and aligns with subwoofer
  Primes & shoots the other note
 */
import frc.robot.subsystems.launcher.RollerLauncherCompetition.LauncherPresets;


public class AutoAmp2 extends AutoBaseClass {

  byte multiplier = 1;

  public AutoAmp2(CrescendoBot robot) {
    super(robot);
  }
  public void start() {
		super.start();
	}  
  public void stop() {
    super.stop();
  }
  public void periodic() {
    if (isRunning()) {
        SmartDashboard.putNumber("Auto Step", getCurrentStep());
        switch (getCurrentStep()) {
            case 0:
              if(DriverStation.getAlliance().get() == Alliance.Blue)
                multiplier = 1;
              if(DriverStation.getAlliance().get() == Alliance.Red)
                multiplier = -1;
              myRobot.launcher.aim(LauncherPresets.AMP);
              myRobot.launcher.setSpeedBias(0.25);
              driveFixedPositionOffsetInches(8*multiplier, -19);
              setTimerAndAdvanceStep(2000);
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
              myRobot.launcher.aim(LauncherPresets.OFF);
              driveFixedPositionOffsetInches(30*multiplier,20);
              myRobot.launcher.load(0.25);
              setTimerAndAdvanceStep(4000);
              break;
            case 5:
              if(myRobot.launcher.isLoaded())
                myRobot.launcher.stopLoader();
              if(myRobot.drive.atFixedPosition(0.5)) {
                advanceStep();
              }
              break;
            case 6:
              myRobot.launcher.aim(LauncherPresets.AMP);
              myRobot.launcher.setSpeedBias(0.25);
              driveFixedPositionOffsetInches(-30*multiplier,-20);
              setTimerAndAdvanceStep(6000);
              break;
            case 7:
              if(myRobot.launcher.isLoaded())
                myRobot.launcher.stopLoader();
              if(myRobot.drive.atFixedPosition(0.5)) {
                advanceStep();
              }
              break;
            case 8:
              myRobot.launcher.fire();
              setTimerAndAdvanceStep(500);
              break;
            case 9:
              break;
            case 10:
              myRobot.launcher.stop();
              stop();
              break;
        }
      }
    }
}
