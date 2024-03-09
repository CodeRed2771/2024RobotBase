// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TuneablePIDGainSet {

    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kIz = 0.0;
    protected String prefix;

    public TuneablePIDGainSet(String prefix) {
        this.prefix = prefix;
    }

    public void postTuneParams() {
        SmartDashboard.putNumber(prefix + " P", this.kP);
        SmartDashboard.putNumber(prefix + " I", this.kI);
        SmartDashboard.putNumber(prefix + " D", this.kD);
        SmartDashboard.putNumber(prefix + " I Zone", this.kIz);
    }

    public void handleTuneParams() {
        // read PID coefficients from SmartDashboard

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        double val = SmartDashboard.getNumber(prefix + " P", this.kP);
        if ((val != this.kP)) {
            setP(val);
        }

        val = SmartDashboard.getNumber(prefix + " I", this.kI);
        if ((val != this.kI)) {
            setI(val);
        }
        val = SmartDashboard.getNumber(prefix + " D", this.kD);
        if ((val != this.kD)) {
            setD(val);
        }
        val = SmartDashboard.getNumber(prefix + " I Zone", this.kIz);
        if ((val != this.kIz)) {
            setIz(val);
        }
    }

    public void setP(double P){kP = P;}
    public void setI(double I){kI = I;}
    public void setIz(double Iz){kIz = Iz;}
    public void setD(double D){kD = D;}
}
