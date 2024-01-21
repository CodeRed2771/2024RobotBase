// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeSubsystem;

public abstract class RobotContainer extends SubsystemBase {

  /* Be sure to register all subsystems after they are created */
  protected IntakeSubsystem intake;

  /** Creates a new RobotContainer and registers the subsystems. */
  protected RobotContainer(String name) {
    super(name);
  }

  protected RobotContainer() {
    super();
  }

  protected void registerSubsystems() {
    this.addChild(intake.getName(), intake);
  }

  /** arm should initialize the robot for motion */
  public abstract void arm();

  /** disarm should stop all subsystems and turn off any motion devices */
  public abstract void disarm();

}
