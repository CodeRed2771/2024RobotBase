package frc.robot.subsystems.drive;

import frc.robot.subsystems.ArmedSubsystem;

public abstract class SwerveModule extends ArmedSubsystem {
    protected SwerveModule() {
        super();
    }

	// override with motor specific class
	public void setDrivePower(double p){}
	public void setDriveMaxAccel(final int accel){}
    public void setDriveMaxVelocity(final int velocity){}
	public void setTurnPower(double p){}
	public double getTurnRelativePosition(){ return 0;}
	public double getTurnAbsolutePosition(){ return 0;}
	public double getTurnPosition(){ return 0;}
	public double getTurnAngle(){ return 0;}
	public boolean modulesReversed(){return false;}
	public void unReverseModule(){}
	public void resetTurnEncoder(){}
	public double getDriveEnc(){ return 0;}
	public void resetDriveEnc(){}
	public void setEncPos(double d){}
	public boolean isTurnEncConnected(){ return false;}
	public int getTurnRotations(){ return 0;}
	public double getTurnOrientation(){ return 0;}
    public double getCurrentDriveSetpoint(){ return 0;}
    public void setDrivePIDToSetPoint(final double setpoint){}
    public boolean hasDriveCompleted(final double inchesError){ return true;}
	public boolean hasDriveCompleted() {
		return hasDriveCompleted(0.25);
	}
	public void setTurnPIDToSetPoint(double setpoint){}
	public void setTurnOrientation(double reqPosition, boolean optimize){}
	public double getTurnError(){return 0;}
	public double getTurnSetpoint(){return 0;}
	public void stopDriveAndTurnMotors(){}
	public void stopDrive(){}
	public void stopTurn(){}
    public void setBrakeMode(final boolean b){}
    public void setDrivePIDValues(final double p, final double i, final double d, final double f){}
	public void setTurnPIDValues(double p, double i, double d, double izone, double f){}
	public double getTurnZero(){return 0;}
	public void resetZeroPosToCurrentPos(){}
	public void resetTurnReversedFlag(){}
	double getABSRotations(){return 0;}
}