// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public abstract class ArmedSubsystem extends SubsystemBase {
    private boolean bArmed = false;


    protected ArmedSubsystem(){
        super();
    }

    public final boolean isArmed(){return bArmed;}
    
    public final void arm(){
        doArm();
        bArmed = true;
    }

    public final void disarm(){
        doDisarm();
        bArmed = false;
    }

    protected abstract void doArm();
    protected abstract void doDisarm();
}
