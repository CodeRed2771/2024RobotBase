// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs;

import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TuneableSparkPIDControllerGains extends TuneablePIDGainSet {

    SparkPIDController controller;

    public double kFF = 0.0;
    public double kMaxVel = 0.0;
    public double kMaxAccel = 0.0;

    public TuneableSparkPIDControllerGains(String prefix, SparkPIDController controller) {
        super(prefix);
        this.controller = controller;
    }

    @Override
    public void postTuneParams(){
        super.postTuneParams();
        SmartDashboard.putNumber(prefix + " FF", this.kFF);
        SmartDashboard.putNumber(prefix + " Max Vel", this.kMaxVel);
        SmartDashboard.putNumber(prefix + " Max Accel", this.kMaxAccel);
    }

    @Override
    public void handleTuneParams() {
        super.handleTuneParams();
        // read PID coefficients from SmartDashboard

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        double val = SmartDashboard.getNumber(prefix + " FF", this.kFF);
        if ((val != this.kFF)) {
            this.kFF = val;
            setFF(val);
        }
        val = SmartDashboard.getNumber(prefix + " Max Vel", this.kMaxVel);
        if ((val != this.kMaxVel)) {
            setMaxVel(val);
        }
        val = SmartDashboard.getNumber(prefix + " Max Accel", this.kMaxVel);
        if ((val != this.kMaxVel)) {
            setMaxAccel(val);
        }
    }

    @Override
    public void setP(double val){super.setP(val); controller.setP(kP);}
    @Override
    public void setI(double val){super.setI(val);controller.setI(kI);}
    @Override
    public void setIz(double val){super.setIz(val);controller.setIZone(kIz);}
    @Override
    public void setD(double val){super.setD(val);controller.setD(kD);}

    public void setFF(double val){kFF = val;controller.setFF(kFF);}
    public void setMaxVel(double val){kMaxVel = val;controller.setSmartMotionMaxVelocity(kMaxVel,0);}
    public void setMaxAccel(double val){kMaxAccel = val;controller.setSmartMotionMaxAccel(kMaxAccel,0);}
}
