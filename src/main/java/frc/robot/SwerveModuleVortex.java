package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModuleVortex implements SwerveModule {
    public CANSparkFlex drive;
    public CANSparkMax turn;
    private SparkPIDController drivePID;
	private SparkPIDController turnPID;
    private RelativeEncoder driveEncoder;
	private RelativeEncoder turnEncoder;
	private AnalogEncoder turnAbsEncoder;
	private char mModuleID;
	// private final int FULL_ROTATION = 4096;
	private double TURN_P, TURN_I, TURN_D, TURN_F, DRIVE_P, DRIVE_I, DRIVE_D;
	private double TURN_IZONE, DRIVE_IZONE;
	private double turnZeroPos = 0;
	private double currentDriveSetpoint = 0;
	private boolean isReversed = false;

	/**
	 * Lets make a new module :)
	 * 
	 * @param driveTalonID First I gotta know what talon we are using for driving
	 * @param turnTalonID  Next I gotta know what talon we are using to turn
	 */
	public SwerveModuleVortex(int driveMotorID, int turnMotorID,  int turnAbsEncID, double tZeroPos, char moduleID) {

        drive = new CANSparkFlex(driveMotorID, MotorType.kBrushless);
        drive.restoreFactoryDefaults();
        drive.setOpenLoopRampRate(.1);
        drive.setSmartCurrentLimit(40);
        drive.setIdleMode(IdleMode.kBrake);
        
        mModuleID = moduleID;

	  /**
         * In order to use PID functionality for a controller, a CANPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        drivePID = drive.getPIDController();

        // Encoder object created to display position values
        driveEncoder = drive.getEncoder();

		DRIVE_P = Calibration.getDriveP();
		DRIVE_I = Calibration.getDriveI();
		DRIVE_D = Calibration.getDriveD();
		DRIVE_IZONE = Calibration.getDriveIZone();

        drivePID.setP(DRIVE_P);
        drivePID.setI(DRIVE_I);
        drivePID.setD(DRIVE_D);
        drivePID.setIZone(DRIVE_IZONE);
        drivePID.setFF(0);
        drivePID.setOutputRange(-1, 1);

        drivePID.setSmartMotionMaxVelocity(Calibration.getDT_MM_VELOCITY(), 0);
        drivePID.setSmartMotionMaxAccel(Calibration.getDT_MM_ACCEL(), 0);

         // TURN
		// turnAbsEncoder = new DutyCycleEncoder(turnAbsEncID);
		turnAbsEncoder = new AnalogEncoder(new AnalogInput(turnAbsEncID));
		

		turn = new CANSparkMax(turnMotorID, MotorType.kBrushless);
		turn.restoreFactoryDefaults();
       // turn.setOpenLoopRampRate(.5);
        turn.setSmartCurrentLimit(30);

        turn.setIdleMode(IdleMode.kBrake);
		turn.setInverted(true);

		// turnEncoder = turn.getAlternateEncoder(8192);
		turnEncoder = turn.getEncoder();
		// turnEncoder.setInverted(true);
		turnPID = turn.getPIDController();
		turnPID.setFeedbackDevice(turnEncoder);

		TURN_P = Calibration.getTurnP();
		TURN_I = Calibration.getTurnI();
		TURN_D = Calibration.getTurnD();
		TURN_IZONE = Calibration.getTurnIZone();
		TURN_F = Calibration.getTurnF();

		setTurnPIDValues(TURN_P, TURN_I, TURN_D, TURN_IZONE, TURN_F);
		
        turnPID.setOutputRange(-1, 1);

		turn.burnFlash(); // save settings for power off

		turnZeroPos = tZeroPos;

		
	}

    public void setDriveMMAccel(final int accel) {
        drivePID.setSmartMotionMaxAccel(accel, 0);
    }
    public void setDriveMMVelocity(final int velocity) {
        drivePID.setSmartMotionMaxVelocity(velocity, 0);
    }

	/**
	 * getModuleLetters
	 * @return a single character, A,B,C,D indicating which module this is
	 */
	public char getModuleLetter() {
		return mModuleID;
	}

	/**
	 * Setting turn motor power
	 * 
	 * @param p value from -1 to 1
	 */
	public void setTurnPower(double p) {
		this.turn.set(p);
	}

	/**
	 * Setting drive motor power
	 * 
	 * @param p value from -1 to 1
	 */
	public void setDrivePower(double p) {
		this.drive.set((isReversed ? -1 : 1) * p);
	}

	/**
	 * Getting the turn encoder position (not absolute)
	 * 
	 * @return turn encoder position
	 */
	public double getTurnRelativePosition() {
		return turnEncoder.getPosition();
	}

	/**
	 * Gets the absolute encoder position for the turn encoder It will be a value
	 * between 0 and 1
	 * 
	 * @return turn encoder absolute position
	 */
	@Override
	public double getABSRotations() {
		return turnAbsEncoder.get();
	}
	public double getTurnAbsolutePosition() {
		double encPos = 0;

		if (turnAbsEncoder.get() >= 0)
			encPos = (turnAbsEncoder.get() - (int) turnAbsEncoder.get()); // e.g. 3.2345 - 3.000 = .2345
		else
			encPos = ((int)Math.abs( turnAbsEncoder.get()) + 1 - Math.abs(turnAbsEncoder.get())); // e.g. -3.7655  = .2345
		// now invert it because the turn motors are inverted
		if (encPos <= .5) 
			encPos = 1 - encPos; // e.g .2 becomes .8
		else
			encPos = .5 - (encPos - .5); // e.g. .5 - (.8 - .5) = .2
		
		return encPos;
	}

	public double getTurnPosition() {
		// returns the 0 to 1 value of the turn position
		// uses the calibration value and the actual position
		// to determine the relative turn position
		return getTurnPositionWithInRotation();
		// double currentPos = getTurnAbsolutePosition();
		// if (currentPos - turnZeroPos > 0) {
		// 	return currentPos - turnZeroPos;
		// } else {
		// 	return (1 - turnZeroPos) + currentPos;
		// }
	}

	public double getTurnAngle() {
		// returns the angle in -180 to 180 range
		double turnPos = getTurnPosition();
		if (turnPos > .5) {
			return (360 - (turnPos * 360));
		} else
			return turnPos * 360;
	}

	public boolean modulesReversed() {
		return isReversed;
	}

	public void unReverseModule() {
		isReversed = false;
	}

	/*
		resets the Quad Encoder based on absolute encoder
	*/
	public void resetTurnEncoder() {
		double currentPos = 0;
		double positionToSet = 0;
		setTurnPower(0);
		Timer.delay(.1); // give module time to settle down
		currentPos = getTurnAbsolutePosition();
		
		positionToSet = calculatePositionDifference(currentPos, turnZeroPos);
		setEncPos(100 + positionToSet);
		// setEncPos(0.95);
	}

	private static double calculatePositionDifference(double currentPosition, double calibrationZeroPosition) {
        if (currentPosition - calibrationZeroPosition >= 0) {
            return currentPosition - calibrationZeroPosition;
        } else {
            return (1 - calibrationZeroPosition) + currentPosition;
        }
    }

	public double getDriveEnc() {
		return driveEncoder.getPosition();
	}

	public void resetDriveEnc() {
        this.driveEncoder.setPosition(0);
	}

	public void setEncPos(double d) {
		turnEncoder.setPosition(d);
	}

	/**
	 * Is electrical good? Probably not.... Is the turn encoder connected?
	 * 
	 * @return true if the encoder is connected
	 */
	public boolean isTurnEncConnected() {
		// return turn.isSensorPresent(FeedbackDevice.CtreMagEncoder_Relative) ==
		// FeedbackDeviceStatus.FeedbackStatusPresent;
		return true; // didn't immediately see a compatible replacement
	}

	public int getTurnRotations() {
		// i have verified that the (int) typecast 
		// does properly truncate off the decimal portion
		// without any rounding.
		return (int) turnEncoder.getPosition();
	}

	public double getTurnOrientation() {
		if (turnEncoder.getPosition() >= 0) {
			return turnEncoder.getPosition() - (int) turnEncoder.getPosition();	
		} else
			return turnEncoder.getPosition() + (int) turnEncoder.getPosition();
	}
	
	public double getTurnPositionWithInRotation() {
		double rawPos = 0;
		if (turnEncoder.getPosition() >= 0) {
			return turnEncoder.getPosition() - (int) turnEncoder.getPosition();	
		} else
		rawPos =  turnEncoder.getPosition() + (int) turnEncoder.getPosition();
		return round(rawPos, 3);
	}
    public double getCurrentDriveSetpoint() {
        return currentDriveSetpoint;
    }

    // These are used for driving and turning in auto.
    public void setDrivePIDToSetPoint(final double setpoint) {
        currentDriveSetpoint = setpoint;
        drivePID.setReference(setpoint, ControlType.kSmartMotion);
    }

	public boolean hasDriveCompleted(final double inchesError) {
        return Math.abs(currentDriveSetpoint - getDriveEnc()) <= Calibration.getDriveTicksPerInch() * inchesError;
    }
      
	public boolean hasDriveCompleted() {
		return hasDriveCompleted(0.25);
	}

	public void setTurnPIDToSetPoint(double setpoint) {
		turn.set(setpoint);
	}

	public void resetTurnReversedFlag() {
		isReversed = false;
	}

		/**
	 * Set turn to pos from 0 to 1 using PID
	 * 
	 * @param reqPosition orientation to set to
	 */
	public void setTurnOrientation(double reqPosition, boolean optimize) {
		// reqPosition - a value between 0 and 1 that indicates the rotational position
		//               that we want the module to be facing.
		// Output
		//      The result of this method is to instruct the turn motor to go to a
		//      position that will equal the desired position.  But that position might
		//      be the opposite side of the circle if we can get there quicker and
		//      just invert the drive direction.
		//      
		// Note
		//		Since "zero" on the modules doesn't equal zero as the requested position
		//      we need to take into consideration the specific zero position of this 
		//		module.  In other words, if the straight ahead "zero" position of this 
		//		module is a reading of ".250" and our requested position is "0", then we
		//		actually have to go to ".250" to satisfy the requested position.
		// 		The encoders actual position will be a number like 5.2332 or -5.3842
		//		indicating that it's 5 revolutions in plus a partial revolution.
		//		so the final turn instruction needs to be relative to that original 
		//      reading otherwise the module will have to unwind a bunch of revolutions.

        boolean invertDrive = false;
        double nearestPosInRotation = 0;
        double newTargetPosition = 0;
		double currentPosition = turnEncoder.getPosition();

		// I think it would be best to adjust our requested position first so that it  
		// is compatible with our modules zero offset.  Then all calculations after that
		// will be in actual encoder positions.

		// reqPosition = round(reqPosition,3); round was crashing occasionally

		//reqPosition += turnZeroPos;  
		if (reqPosition > 0.99999) // we went past the rotation point
			reqPosition -= 1;  // remove the extra rotation. change 1.2344 to .2344
        double reqPositionReverse = (reqPosition >= .5 ? reqPosition - .5 : reqPosition + .5) ; // e.g. .8 becomes .3

		// now we can check where we currently are and figure out the optimal new position 
		// based on which of the two potential positions is closest.
		double currentPosInRotation = getTurnPositionWithInRotation();  // this is where it is .
		int currentRevolutions = getTurnRotations();  // remember this for later

        // e.g. curpos = .125;  req = .800  rot=15

		if (optimize) {
			// if the difference between the current position and the requested position is less than 
			// a half rotation, then use that position, otherwise use the reverse position
			if (Math.abs(currentPosInRotation - reqPosition) <= .25 || Math.abs(currentPosInRotation - (1 + reqPosition)) <= .25|| Math.abs((currentPosInRotation+1)-reqPosition) <=0.25) {
				nearestPosInRotation = reqPosition;
				invertDrive = false;
			} else {
				nearestPosInRotation = reqPositionReverse;
				invertDrive = true;
			}
		} else {
			nearestPosInRotation = reqPosition;
			invertDrive = false;
		}
        
        
        // now we need to determine if we need to change our rotation counter to get to 
        // this new position
        // if we're at .9 and need to get to .1 for instance, then we need to increment 
        // our rotation count or the module will unwind backwards a revolution.

        // e.g. curpos = .125;  nearest = .300  rot=15
        // have to handle cur = .125 nearest = .99  rot=15 
        // and have to handle negative rotations
        double newRevolutions = currentRevolutions;
        if (nearestPosInRotation < currentPosInRotation)   {
            // our nearest is a smaller number so we may need to change revolutions
            // if the difference is larger than .5 then we are rotating across
            // the zero position, so we need to adjust revolutions
            if (Math.abs(currentPosInRotation - nearestPosInRotation) > .5) {
                // e.g cur = .9  near = .1 rot = 15, then new = 16.1
                // e.g cur = .9  near = .1 rot = -15, then new = -16.1
                newRevolutions = (currentRevolutions >= 0 ? currentRevolutions + 1 : currentRevolutions - 1);
            } 
        } else { 
            // see if we're backing up across the zero and then reduce the revs
            if (Math.abs(currentPosInRotation - nearestPosInRotation) > .5) {
                // e.g cur = .9  near = .1 rot = 15, then new = 16.1
                // e.g cur = .9  near = .1 rot = -15, then new = -16.1
                newRevolutions = (currentRevolutions >= 0 ? currentRevolutions - 1 : currentRevolutions + 1);
            } 
        }

        newTargetPosition = (newRevolutions >= 0 ? newRevolutions + nearestPosInRotation : newRevolutions - nearestPosInRotation);
        
		// newTargetPosition = round(newTargetPosition,3);  round was crashing occasionally

		// Set drive inversion if needed
		isReversed = invertDrive; // invert

        // TURN
        turnPID.setReference(newTargetPosition, ControlType.kPosition );

		// if (mModuleID=='C') {
		// 	SmartDashboard.putString("MC Cur Pos", String.format("%.3f", turnEncoder.getPosition()));
		// 	SmartDashboard.putString("MC Req Pos", String.format("%.3f", reqPosition));
		// 	SmartDashboard.putString("MC Nearest", String.format("%.3f", nearestPosInRotation));
		// 	SmartDashboard.putString("MC NewTarget", String.format("%.3f", newTargetPosition));
		// 	SmartDashboard.putBoolean("MC Reversed", isReversed);
		// }

		// look for error
		// if (Math.abs(currentPosition - newTargetPosition) > .250) {
		// 	SmartDashboard.putString("MOD " + mModuleID + " CALC ERROR", "Cur: " + String.format("%.3f", currentPosition) + " New: " + String.format("%.3f", newTargetPosition) + "Called:  " + String.format("%.3f", reqPosition));
		// }

        // System.out.println("");
        // System.out.println("Current Rotations:" + currentRevolutions);
        // System.out.println("Current Position: " + currentPosInRotation);
        // System.out.println("Requested Pos:    " + reqPosition);
        // System.out.println("Requested Pos Rev:" + reqPositionReverse);
        // System.out.println("Nearest Pos:      " + nearestPosInRotation);
		// System.out.println("NEW TARGET POS:   " + newTargetPosition);
        // System.out.println("INVERT DRIVE:     " + invertDrive);
	}



	public double getTurnError() {
		return 9999; // fix later
	}

	public double getTurnSetpoint() {
		return 9999; // fix later
	}

	// public double getDriveError() {
		// note that when using Motion Magic, the error is not what you'd expect
		// MM sets intermediate set points, so the error is just the error to
		// that set point, not to the final setpoint.
	// 	return drive.getClosedLoopError(0);
	// }

	public void stopDriveAndTurnMotors() {
		setDrivePower(0);
		setTurnPower(0);
	}

	public void stopDrive() {
		setDrivePower(0);
	}

	public void stopTurn() {
		setTurnPower(0);
	}

    public void setBrakeMode(final boolean b) {
        drive.setIdleMode(b ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void setDrivePIDValues(final double p, final double i, final double d, final double f) {
        drivePID.setP(p);
        drivePID.setI(i);
        drivePID.setD(d);
        drivePID.setFF(f);
    }

	public void setTurnPIDValues(double p, double i, double d, double izone, double f) {
		turnPID.setP(p);
        turnPID.setI(i);
        turnPID.setIZone(izone);
        turnPID.setD(d);
        turnPID.setFF(f);
	}

	public double getTurnZero() {
		return turnZeroPos;
	}
	
	public void resetZeroPosToCurrentPos() {
		
	}

	public static Double round(Double val, int scale) {
		try {
			return new BigDecimal(val.toString()).setScale(scale, RoundingMode.HALF_UP).doubleValue();
		} catch (Exception ex) {
			return val;
		}

    }
}