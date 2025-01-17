// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** This class creates an empty class so that robots do no have to implement all subsystems. */
public class RollerIntake extends IntakeSubsystem {
    
    private CANSparkMax driveMotor;
    
    public RollerIntake(Map<String,Integer> wiring) {
        super();

        int motorId = wiring.get("intakeMotorId");

        driveMotor = new CANSparkMax(motorId, MotorType.kBrushless);
    }

    @Override
    public void doDisarm() {
        stop();
    }

    @Override
    public void load() {
        if(isDisarmed()){
            return;
        }
        driveMotor.set(-.75);
    }

    @Override
    public void stop() {
        driveMotor.set(0);
    }

    @Override
    public void unload() {
        if(isDisarmed()){
            return;
        }
        driveMotor.set(.5);
    }

}
