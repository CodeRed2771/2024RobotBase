// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Map;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmedSubsystem;

/** Add your docs here. */
public class TankWheelVortex extends ArmedSubsystem {

    private CANSparkFlex m_driveMotor;
    private double direction = 1.0;
    public TankWheelVortex(Map<String,Integer> wiring, Map<String,Double> calibration, String moduleID){

        this.setName(moduleID);

        int DT_DRIVE_ID = wiring.get( moduleID + " drive");
        m_driveMotor = new CANSparkFlex(DT_DRIVE_ID, MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();
        Timer.delay(0.5);

        direction = calibration.getOrDefault(moduleID + " direction", 1.0);
        m_driveMotor.setInverted(direction < 0.0);
    }

    public void commandSpeed(double fwd){
        m_driveMotor.set(direction*fwd);
    }
}
