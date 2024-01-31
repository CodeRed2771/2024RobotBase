// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/** This class creates an empty class so that robots do no have to implement all subsystems. */
public class DummyIntake extends IntakeSubsystem {
    public DummyIntake() {
        super();
    }

    private void log(String text){
        System.out.println(getName() + " : " + text);
    }

    public void doArm() {
        log("Armed");
    }

    public void doDisarm() {
        log("Disarmed");
    }

    public void load() {
        //log("Running load actuators");
    }

    public void stop() {
        //log("stop motions");
    }

    public void unload() {
        //log("Unloading element");
    }

}
