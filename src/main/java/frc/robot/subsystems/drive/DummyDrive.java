// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

/** Add your docs here. */
public class DummyDrive extends DriveSubsystem {
    public DummyDrive() {
        super();
    }

    public DummyDrive(String name) {
        super(name);
    }

    private void log(String text) {
        System.out.println(getName() + " : " + text);
    }

    public void arm() {
        log("Armed");
    }

    public void disarm() {
        log("Disarmed");
    }

}
