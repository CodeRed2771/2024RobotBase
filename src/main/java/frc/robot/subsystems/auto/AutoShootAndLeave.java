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
              myRobot.launcher.prime(LauncherSpeeds.OFF);
              myRobot.launcher.stop();
              setTimerAndAdvanceStep(9000); // wait till end of auto so we don't get in the way
              break;
            case 5:
              break;
            case 6:
              if (position=='C')
                myRobot.drive.driveFixedPositionOffsetInches(60,0);
              else 
                myRobot.drive.driveFixedPositionOffsetInches(36, 0);
                
              setTimerAndAdvanceStep(4000);
              break;
            case 7:
              if(myRobot.drive.atFixedPosition(0.5)) {
                advanceStep();
              }
              break;
            case 8:
              if (position=='C') 
                setStep(99); // jump to end of program
              else 
                // do final moves for side positions
                advanceStep();
              break;
            case 9: // these steps are for the side positions only
              if (position=='R') // strafe left
                myRobot.drive.driveFixedPositionOffsetInches(0, -50);
              else //strafe right
                myRobot.drive.driveFixedPositionOffsetInches(0, 50);
              setTimerAndAdvanceStep(4000);
              break;
            case 10:
              if(myRobot.drive.atFixedPosition(0.5)) {
                advanceStep();
              }
              break;
            case 11:
              setStep(99);
              break;
            case 99:
              stop();
              break;
        }
      }
    }
}
