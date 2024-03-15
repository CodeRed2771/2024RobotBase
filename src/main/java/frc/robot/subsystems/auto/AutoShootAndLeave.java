// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auto;

import java.util.Optional;
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
import frc.robot.subsystems.launcher.RollerLauncher.LauncherSpeeds;


public class AutoShootAndLeave extends AutoBaseClass {

  CrescendoBot myRobot;
  private char position = 'C';
  private Optional<Alliance> alliance;

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
              setTimerAndAdvanceStep(1000); // wait till end of auto so we don't get in the way
              break;
            case 5:
              // if (DriverStation.getMatchTime() <= 4) // make sure we get going within 4 seconds left
              //   advanceStep();
              break;
            case 6:
              if (position=='C')
                myRobot.drive.driveFixedPositionOffsetInches(60,0);
              else 
                if (position=='A') //amp side
                  myRobot.drive.driveFixedPositionOffsetInches(50, -12);
                else 
                  myRobot.drive.driveFixedPositionOffsetInches(60, 15); // drive further on source side
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
            //
            // these steps are for the side positions only
            //
            case 9: 
              if (position=='A') // amp
                if (alliance.get()==Alliance.Blue) 
                  // go to the right to leave
                  myRobot.drive.driveFixedPositionOffsetInches(12, -60);
                else 
                  // go to the left to leave
                  myRobot.drive.driveFixedPositionOffsetInches(35, 57);
              else // source side
                if (alliance.get()==Alliance.Blue) 
                  // go to the left to leave
                  myRobot.drive.driveFixedPositionOffsetInches(12, 50);
                else 
                  // go to the right to leave
                  myRobot.drive.driveFixedPositionOffsetInches(12, -50);

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
