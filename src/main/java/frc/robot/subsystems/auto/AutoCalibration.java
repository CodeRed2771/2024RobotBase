// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auto;

import frc.robot.CrescendoBot;
/*
  This auto just drives forward 3ft and then back
 */
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoCalibration extends AutoBaseClass {

  public AutoCalibration(CrescendoBot robot) {
    super(robot);
  }

  public void periodic() {
    if (isRunning()) {
        SmartDashboard.putNumber("Auto Step", getCurrentStep());
        switch (getCurrentStep()) {
            case 0:
              myRobot.drive.driveFixedPositionOffsetInches(36,36);
              setTimerAndAdvanceStep(8000);
              break;
            case 1:
              if(myRobot.drive.atFixedPosition(0.5)) {
                advanceStep();
              }
              break;
            case 2:
              myRobot.drive.driveFixedPositionOffsetInches(-36,-36);
              setTimerAndAdvanceStep(8000);
              break;
            case 3:
              if(myRobot.drive.atFixedPosition(0.5)) {
                advanceStep();
              }
              break;
            case 4:
              myRobot.drive.driveFixedPositionOffsetInches(36,0);
              setTimerAndAdvanceStep(8000);
              break;
            case 5:
              if(myRobot.drive.atFixedPosition(0.5)) {
                advanceStep();
              }
              break;
            case 6:
              myRobot.drive.driveFixedPositionOffsetInches(-36,0);
              setTimerAndAdvanceStep(8000);
              break;
            case 7:
              if(myRobot.drive.atFixedPosition(0.5)) {
                advanceStep();
              }
              break;
            case 8:
              myRobot.drive.driveFixedRotatePosition(90);
              setTimerAndAdvanceStep(8000);
              break;
            case 9:
              if(myRobot.drive.atFixedPosition(0.5)) {
                advanceStep();
              }
              break;
            case 10:
              myRobot.drive.driveFixedRotatePosition(-90);
              setTimerAndAdvanceStep(8000);
              break;
            case 11:
              if(myRobot.drive.atFixedPosition(0.5)) {
                advanceStep();
              }
              break;
            default:
              break;
        }
      }
    }
}
