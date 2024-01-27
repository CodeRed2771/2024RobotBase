// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

/** Add your docs here. */
public class DummyLauncher extends LauncherSubsystem {
    public DummyLauncher() {
        super();
    }

    private void log(String text) {
        System.out.println(getName() + " : " + text);
    }

    public void doArm() {
        log("Armed");
    }

    public void doDisarm() {
        log("Disarmed");
    }

    public void load() {
        log("Running load actuators");
    }

    public void fire(double power) {
        log("firing element at " + (power * 100) + "%");
    }

}