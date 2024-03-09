// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TuneableSlewRateLimiter extends DynamicSlewRateLimiter {
    protected String prefix;
    protected boolean symetric = false;

    public TuneableSlewRateLimiter(String prefix, double positiveRateLimit, double negativeRateLimit, double initialValue) {
        super(positiveRateLimit,negativeRateLimit,initialValue);
        this.prefix = prefix;
    }

    public TuneableSlewRateLimiter(String prefix, double rateLimit) {
        super(rateLimit);
        this.prefix = prefix;
        symetric =true;
    }

    public void postTuneParams() {
        if(symetric){
            SmartDashboard.putNumber(prefix + " rate", this.m_positiveRateLimit);
        } else {
            SmartDashboard.putNumber(prefix + " +rate", this.m_positiveRateLimit);
            SmartDashboard.putNumber(prefix + " -rate", this.m_negativeRateLimit);
        }
    }

    public void handleTuneParams() {
        if(symetric){
            double posRate = SmartDashboard.getNumber(prefix + " rate", this.m_positiveRateLimit);
            if ((posRate != this.m_positiveRateLimit)) {
                setPositiveRate(posRate);
                setNegativeRate(-posRate);
            }
        } else {
            double posRate = SmartDashboard.getNumber(prefix + " +rate", this.m_positiveRateLimit);
            if ((posRate != this.m_positiveRateLimit)) {
                setPositiveRate(posRate);
            }

            double negLimit =  SmartDashboard.getNumber(prefix + " -rate", this.m_negativeRateLimit);
            if ((negLimit != this.m_negativeRateLimit)) {
                setNegativeRate(negLimit);
            }
        }
    }
}
