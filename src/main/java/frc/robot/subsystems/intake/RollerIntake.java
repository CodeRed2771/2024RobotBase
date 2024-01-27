// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** This class creates an empty class so that robots do no have to implement all subsystems. */
public class RollerIntake extends IntakeSubsystem {
    
    private CANSparkMax driveMotor;
    
    
    
    public RollerIntake(int motorId) {
        super();
        CreateIntake(motorId);
    }

    public RollerIntake(String name, int motorId) {
        super(name);
        CreateIntake(motorId);
    }

    private void CreateIntake(int motorId){
        driveMotor = new CANSparkMax(motorId, MotorType.kBrushless);
        armed = false;
    }

    private void log(String text){
        System.out.println(getName() + " : " + text);
    }

    public void arm() {
        armed = true;
    }

    public void disarm() {
        stop();
        armed = false;
    }

    public void load() {
        if(!armed){
            return;
        }
        driveMotor.set(1.0);
    }

    public void stop() {
        if(!armed){
            return;
        }
        driveMotor.set(0);
    }

    public void unload() {
        if(!armed){
            return;
        }
        driveMotor.set(-1.0);
    }

}
