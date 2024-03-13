// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auto;

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
import frc.robot.subsystems.launcher.RollerLauncher.LauncherSpeeds;


public class AutoShootAndLeave extends AutoBaseClass {

  CrescendoBot myRobot;
  private int drivenTicks = 0;
  private char position = 'C';

  public AutoShootAndLeave(CrescendoBot robot) {
    super();
    myRobot = robot;
  }

  public AutoShootAndLeave(CrescendoBot robot, char position) {
    super();
    myRobot = robot;
    this.position = position;
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
              myRobot.launcher.prime(LauncherSpeeds.SUBWOOFER);
              setTimerAndAdvanceStep(800);
              break;
            case 1:
              break;
            case 2:
              myRobot.launcher.fire();
              setTimerAndAdvanceStep(250);
              break;
            case 3:
              break;
            case 4:
              myRobot.launcher.prime(LauncherSpeeds.OFF);
              if (position=='C')
                myRobot.drive.driveFixedPositionOffsetInches(60,0);
              else 
                myRobot.drive.driveFixedPositionOffsetInches(30, 0);
                
              setTimerAndAdvanceStep(4000);
              break;
            case 5:
              if(myRobot.drive.atFixedPosition(0.5)) {
                advanceStep();
              }
              break;
            case 6:
              advanceStep();
              break;
            case 7:
              myRobot.launcher.stop();
              stop();
              break;
        }
      }
    }
}
