// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LauncherSubsystem extends SubsystemBase {
  protected LauncherSubsystem() {
    super();
  }

  protected LauncherSubsystem(String name) {
    super(name);
  }

  public abstract void arm();
  public abstract void disarm();

  /** request that the intake perform the operations to try and load a new element into the robot */
  public abstract void load();

  public abstract void fire(double power);
}
