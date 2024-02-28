// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DefaultRobot;
import frc.robot.PracticeRobot;
/*
  This auto (that starts in the center position):
  Primes & shoots 1 note,
  Drives forward and picks up another note (Leaving the starting area in the process)
  Drives back and aligns with subwoofer
  Primes & shoots the other note
 */


public class AutoShoot2Center extends AutoBaseClass {

  PracticeRobot myRobot;
  private int drivenTicks = 0;

  public AutoShoot2Center(PracticeRobot robot) {
    super();
    myRobot = robot;
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
            // case 0:
            //   myRobot.launcher.prime(0.3);
            //   setTimerAndAdvanceStep(750);
            //   break;
            // case 1:
            //   break;
            // case 2:
            //   myRobot.launcher.fire();
            //   setTimerAndAdvanceStep(250);
            //   break;
            // case 3:
            //   break;
            case 0:
              // myRobot.launcher.stop();
              myRobot.drive.driveInches(10,0.2,0);//90
              // myRobot.launcher.load(0.45);
              setTimerAndAdvanceStep(4000);
              break;
            case 1:
              if(myRobot.drive.driveCompleted(0.5))
                advanceStep();
              break;
            // case 6:
            //   myRobot.launcher.prime(0.3);
            //   myRobot.drive.driveInches(0,0.7,0);//-92
            //   setTimerAndAdvanceStep(6000);
            //   break;
            // case 7:
            //   if(myRobot.drive.driveCompleted(0.5))
            //     advanceStep();
            //   break;
            // case 8:
            //   myRobot.launcher.fire();
            //   break;
            case 2:
              myRobot.launcher.stop();
              stop();
              break;
        }
      }
    }
}
