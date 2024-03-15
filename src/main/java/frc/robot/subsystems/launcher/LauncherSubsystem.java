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
  // public void load(){loadState = LoaderState.Loading;}
  public void load(double power){loadState = LoaderState.Loading;}
  public void unload() {loadState = LoaderState.Unloading;}
  public void stopLoader() {loadState = LoaderState.Stopped;}
  public boolean isLoaded() {return true;}
  public enum LoaderState {
    Loading, 
    Firing,
    Stopping,
    Stopped,
    Unloading
  } 
  protected LoaderState loadState;
  public LoaderState getLoaderState() {return loadState;}

  public boolean isAimed() {return true;}
  public void aim(double angle){}

  public void prime(double power){}
  public void stop() {prime(0);}
  public boolean isPrimed() {return true;}
  public boolean isFiring() {return false;}
  public boolean isUnloading() {return false;}
  public void fire() {}
  public void reset() {}
}
