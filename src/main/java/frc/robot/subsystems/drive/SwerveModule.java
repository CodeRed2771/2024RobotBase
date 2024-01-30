package frc.robot.subsystems.drive;

public interface SwerveModule {

	// override with motor specific class
	public void setDrivePower(double p);
	public void setDriveMaxAccel(final int accel);
    public void setDriveMaxVelocity(final int velocity);
	public void setTurnPower(double p);
	public double getTurnRelativePosition();
	public double getTurnAbsolutePosition();
	public double getTurnPosition();
	public double getTurnAngle();
	public boolean modulesReversed();
	public void unReverseModule();
	public void resetTurnEncoder();
	public double getDriveEnc();
	public void resetDriveEnc();
	public void setEncPos(double d);
	public boolean isTurnEncConnected();
	public int getTurnRotations();
	public double getTurnOrientation();
    public double getCurrentDriveSetpoint();
    public void setDrivePIDToSetPoint(final double setpoint);
    public boolean hasDriveCompleted(final double inchesError);
	public boolean hasDriveCompleted();
	public void setTurnPIDToSetPoint(double setpoint);
	public void setTurnOrientation(double reqPosition, boolean optimize);
	public double getTurnError();
	public double getTurnSetpoint();
	public void stopDriveAndTurnMotors();
	public void stopDrive();
	public void stopTurn();
    public void setBrakeMode(final boolean b);
    public void setDrivePIDValues(final double p, final double i, final double d, final double f);
	public void setTurnPIDValues(double p, double i, double d, double izone, double f);
	public double getTurnZero();
	public void resetZeroPosToCurrentPos();
	public void resetTurnReversedFlag();
	double getABSRotations();
}