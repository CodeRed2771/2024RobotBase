// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import frc.robot.subsystems.ArmedSubsystem;

public abstract class LauncherSubsystem extends ArmedSubsystem {
  protected LauncherSubsystem() {
    super();
  }

  /** request that the intake perform the operations to try and load a new element into the robot */
  public void load(){}

  public void fire(double power){}
}
