// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs;

import edu.wpi.first.math.controller.PIDController;

public class TuneablePIDControllerGains extends TuneablePIDGainSet {

    PIDController controller;

    public TuneablePIDControllerGains(String prefix, PIDController controller) {
        super(prefix);
        this.controller = controller;
    }


    @Override
    public void setP(double val){super.setP(val); controller.setP(kP);}
    @Override
    public void setI(double val){super.setI(val);controller.setI(kI);}
    @Override
    public void setIz(double val){super.setIz(val);controller.setIZone(kIz);}
    @Override
    public void setD(double val){super.setD(val);controller.setD(kD);}

}
