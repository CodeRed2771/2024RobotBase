// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.subsystems.ArmedSubsystem;
/**
 * The Intake subsystem provides generic calls that all Intake Subsystems have to support. The subsystem owner should
 * only interact with this API The typical sequence of Automatic events would be: Start the robot Disarmed On match
 * start or operator command, Arm intake to acquire elements If an element reaches the intake, the intake is operated
 * until an element is loaded into the robot (acquired) On being loaded, the intake will stop operating until the
 * element is removed through the unload() indcating a transfer to another part of the robot. The intake should reject
 * loading any other elements until the current element is removed. If the Intake is not automatic, the operator can
 * command load which will operate the intake until an element is loaded If the Intake can detect element being fully
 * loaded, the
 */
public abstract class IntakeSubsystem extends ArmedSubsystem {
  protected IntakeSubsystem() {
    super();
  }

  public void deploy(){}
  public void stow(){}
  public boolean isDeployed(){return true;}

  /** request that the intake perform the operations to try and load a new element into the robot */
  public void load(){}
  /** request that the intake perform the operations to try and remove an element from the robot */
  public void unload(){}
  /** request that the stop performing any motions */
  public void stop(){}

  public boolean hasElement(){return false;}

}
