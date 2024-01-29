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
    public final boolean isDisarmed(){return !bArmed;}
    
    /** While armed a ArmedSubsystem is likely to move and potentially cause injury. */
    public final void arm(){
        /* Ignore repeated arm calls */
        if (isArmed()){return;}
        doArm();
        bArmed = true;
    }

    /** While disarmed, the ArmedSubsystem should not be able to move or cause injury. */
    public final void disarm(){
        /* Ignore repeated disarm calls */
        if (isDisarmed()){return;}
        doDisarm();
        bArmed = false;
    }

    protected void doArm(){}
    protected void doDisarm(){}
}
