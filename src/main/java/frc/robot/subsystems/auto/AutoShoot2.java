// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DefaultRobot;
import frc.robot.PracticeRobot;

public class AutoShoot2 extends AutoBaseClass {

  PracticeRobot myRobot;
  private int drivenTicks = 0;

  public AutoShoot2(PracticeRobot robot) {
    super();
    myRobot = robot;
  }
  public void start() {
		super.start();
	}  
  public void stop() {
    super.stop();
  }
  public void tick() {
    if (isRunning()) {
        SmartDashboard.putNumber("Auto Step", getCurrentStep());
        switch (getCurrentStep()) {
              case 0:
              myRobot.launcher.prime(0.3);
              setTimerAndAdvanceStep(750);
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
              myRobot.launcher.stop();
              myRobot.drive.driveInches(114,0.7,0);
              myRobot.launcher.load(0.45);
              setTimerAndAdvanceStep(6000);
              break;
            case 5:
              if(myRobot.drive.driveCompleted(0.5))
                advanceStep();
              break;
            case 6:
              myRobot.launcher.prime(0.3);
              myRobot.drive.driveInches(-114,0.7,0);
              setTimerAndAdvanceStep(6000);
              break;
            case 7:
              if(myRobot.drive.driveCompleted(0.5))
                advanceStep();
              break;
            case 8:
              myRobot.launcher.fire();
              break;
            case 9:
              myRobot.launcher.stop();
              stop();
              break;
        }
      }
    }
}
