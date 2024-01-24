package frc.robot;

public interface SwerveModule {

	// override with motor specific class
	
	void setDrivePower(double p);
	void setDriveMMAccel(final int accel);
    void setDriveMMVelocity(final int velocity);
	char getModuleLetter();
	void setTurnPower(double p);
	double getTurnRelativePosition();
	double getTurnAbsolutePosition();
	double getTurnPosition();
	double getTurnAngle();
	boolean modulesReversed();
	void unReverseModule();
	void resetTurnEncoder();
	double getDriveEnc();
	void resetDriveEnc();
	void setEncPos(double d);
	boolean isTurnEncConnected();
	int getTurnRotations();
	double getTurnOrientation();
    double getCurrentDriveSetpoint();
    void setDrivePIDToSetPoint(final double setpoint);
    boolean hasDriveCompleted(final double inchesError);
	boolean hasDriveCompleted();
	void setTurnPIDToSetPoint(double setpoint);
	void setTurnOrientation(double reqPosition, boolean optimize);
	double getTurnError();
	double getTurnSetpoint();
	void stopDriveAndTurnMotors();
	void stopDrive();
	void stopTurn();
    void setBrakeMode(final boolean b);
    void setDrivePIDValues(final double p, final double i, final double d, final double f);
	void setTurnPIDValues(double p, double i, double d, double izone, double f);
	double getTurnZero();
	void resetZeroPosToCurrentPos();
	void resetTurnReversedFlag();
	double getABSRotations();
}