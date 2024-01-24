package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.libs.CurrentBreaker;

public class DriveAuto {
    private static PIDController rotDrivePID;

    private static boolean isDriving = false;
    private static boolean followingTarget = false;

    private static double heading = 0; // keeps track of intended heading - used for driving "straight"
	private static double strafeAngle = 0;
	private static double GyroAngle = 0;
    private static double strafeAngleOriginal = 0;

    // private static CurrentBreaker driveCurrentBreaker;

    public static enum DriveSpeed {
        VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED
    };

    public static class Position {
        double x;
        double y;
        double orientation;
        public Position(double x, double y, double orientation) {
            this.x=x;
            this.y=y;
            this.orientation=orientation;
        }
    }

    public static Position currrentPosition; 

    public static void init() {
        System.out.println("START OF DRIVEAUTO INIT");

        rotDrivePID = new PIDController(Calibration.AUTO_ROT_P, Calibration.AUTO_ROT_I, Calibration.AUTO_ROT_D);
        rotDrivePID.setTolerance(2); // degrees off

        DriveTrain.setDriveMMAccel(Calibration.getDT_MM_ACCEL());
        DriveTrain.setDriveMMVelocity(Calibration.getDT_MM_VELOCITY());

        // driveCurrentBreaker = new CurrentBreaker(Wiring.DRIVE_PDP_PORT, 55, 400); 
        // driveCurrentBreaker.reset();

        SmartDashboard.putNumber("AUTO DRIVE P", Calibration.getDriveP());
        SmartDashboard.putNumber("AUTO DRIVE I", Calibration.getDriveI());
        SmartDashboard.putNumber("AUTO DRIVE D", Calibration.getDriveD());
        SmartDashboard.putNumber("AUTO DRIVE F", Calibration.getDriveF());

        SmartDashboard.putNumber("DRIVE MM VELOCITY", Calibration.getDT_MM_VELOCITY());
        SmartDashboard.putNumber("DRIVE MM ACCEL", Calibration.getDT_MM_ACCEL());

        SmartDashboard.updateValues();

        System.out.println("END OF DRIVEAUTO INIT");
        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            if (DriverStation.getLocation().getAsInt() == 1) {
                currrentPosition = new Position(-5, -3, 15);
            } else if (DriverStation.getLocation().getAsInt() == 2) {
                currrentPosition = new Position(0, -3, 0);
            } else {
                currrentPosition = new Position(5, -3, -15);
            }
        } else {
            if (DriverStation.getLocation().getAsInt() == 1) {
                currrentPosition = new Position(-5, 3, 15);
            } else if (DriverStation.getLocation().getAsInt() == 2) {
                currrentPosition = new Position(0, 3, 0);
            } else {
                currrentPosition = new Position(5, 3, -15);
            }
        }
	}

    private static void calculateNewDrivePosition(double distance, double angle) {
        double slope;
        if (distance > 0) {
            slope = Math.tan(angle);
        } else {
            slope = Math.tan(-angle);
        }
        double x =1;
        double y = slope*(x+currrentPosition.x) - currrentPosition.y;
    }

    private static void calcualteNewTurnPosition(double degrees) {
        currrentPosition.orientation = currrentPosition.orientation + degrees;
    }

	public static void driveInches(double inches, double angle, double speedFactor, boolean followTarget) {
		driveInches(inches, angle, speedFactor, followTarget, false);
	}

    public static void driveInches(double inches, double angle, double speedFactor, boolean followTarget, boolean fieldCentric) {

        followingTarget = followTarget;

        SmartDashboard.putNumber("DRIVE INCHES", inches);

		if (fieldCentric) {
			GyroAngle = RobotGyro.getRelativeAngle();
			strafeAngle = angle;
			strafeAngleOriginal = strafeAngle;

			if (GyroAngle > 0 && GyroAngle < 180) {
				strafeAngle = strafeAngle + GyroAngle;
			} else if (GyroAngle > 180 && GyroAngle < 360) {
				GyroAngle = 360 - GyroAngle;
				strafeAngle = strafeAngle - GyroAngle;
			}

		} else {
			strafeAngle = angle;
			strafeAngleOriginal = strafeAngle;
		}
		
        stopTurning();

        isDriving = true;

        DriveTrain.setDriveMMVelocity((int) (Calibration.getDT_MM_VELOCITY() * speedFactor));

        // angle at which the wheel modules should be turned

        // didnt help - DriveTrain.unReverseModules(); // make sure all "reversed" flags
        // are reset.
        SmartDashboard.putNumber("Strafe Angle:", strafeAngle);
        DriveTrain.setAllTurnOrientation(DriveTrain.angleToPosition(strafeAngle), true);

        // give it just a little time to get the modules turned to position
        // before starting the drive
        // this helps to get accurate encoder readings too since the drive
        // encoder values are affected by turning the modules
        try {
            Thread.sleep(150);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        // set the new drive distance setpoint
        DriveTrain.addToAllDrivePositions(convertToTicks(inches));
        calculateNewDrivePosition(inches, angle);
    }

    public static void driveInches(double inches, double angle, double speedFactor) {
        driveInches(inches, angle, speedFactor, false);
    }

    public static void reset() {
        stop();
        DriveTrain.resetDriveEncoders();
        rotDrivePID.reset();
        rotDrivePID.setSetpoint(0);
        heading = RobotGyro.getRelativeAngle();
        isDriving = false;
    }

    public static void stop() {
        stopTurning();
        stopDriving();
    }

    public static void stopDriving() {
        isDriving = false;
        DriveTrain.stopDriveAndTurnMotors();
    }

    public static void stopTurning() {
        // rotDrivePID.setSetpoint(rotDrivePID.calculate(RobotGyro.getAngle())); //
        // changed 1/6/20
        // rotDrivePID.disable();
        // DriveTrain.stopDriveAndTurnMotors();
        DriveTrain.stopTurn();
    }

    public static void setTurnDegreesToCurrentAngle() {
        // this is necessary so that subsequent turns are relative to the current
        // position. Otherwise they'd always be relative to 0
        rotDrivePID.setSetpoint(RobotGyro.getAngle());
    }

    public static double degreesToInches(double degrees) {
        double inches = degrees / 4.42; //3.115;
        return inches;
    }

    public static void turnToHeading(double desiredHeading, double turnSpeedFactor) {
        // double turnAmount = desiredHeading - RobotGyro.getRelativeAngle();

        double turnAmount = 0;

        if (desiredHeading - RobotGyro.getRelativeAngle() > 180)
        {
            turnAmount = (desiredHeading - RobotGyro.getRelativeAngle()) - 360;   
        } 
        else if (desiredHeading - RobotGyro.getRelativeAngle() < -180)
        {
            turnAmount = (desiredHeading - RobotGyro.getRelativeAngle()) + 360; 
        }
        else
        {
            turnAmount = desiredHeading - RobotGyro.getRelativeAngle();
        }
        SmartDashboard.putNumber("DESIRED HEADING EQUALS", desiredHeading);
        SmartDashboard.putNumber("ROBOT GYRO RELATIVE", RobotGyro.getRelativeAngle());
        SmartDashboard.putNumber("TURN AMOUNT", turnAmount);
        turnDegrees(turnAmount, turnSpeedFactor);
        SmartDashboard.putNumber("NEW ROBOT GYRO ANGLE", RobotGyro.getRelativeAngle());
    }

    public static void turnDegrees(double degrees, double turnSpeedFactor) {
        // Turns by driving with the modules turned.
        // Use "turnCompleted" method to determine when the turn is done
        // The PID controller for this sends a rotational value to the
        // standard swerve drive method to make the bot rotate

        stopDriving();

        heading += degrees; // this is used later to help us drive straight after rotating

        SmartDashboard.putNumber("TURN DEGREES CALL", degrees);

        DriveTrain.setTurnOrientation(DriveTrain.angleToPosition(-133.6677), DriveTrain.angleToPosition(46.3322),
                DriveTrain.angleToPosition(133.6677), DriveTrain.angleToPosition(-46.3322), true);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        DriveTrain.setDriveMMVelocity((int) (Calibration.getDT_MM_VELOCITY() * turnSpeedFactor));

        double turnInches = degreesToInches(-degrees);
        SmartDashboard.putNumber("Turn Inches", turnInches);
        DriveTrain.addToAllDrivePositions(convertToTicks(turnInches));
        calcualteNewTurnPosition(degrees);
    }

    public static void continuousDrive(double inches, double maxPower) {
        // setRotationalPowerOutput(maxPower);

        DriveTrain.setTurnOrientation(DriveTrain.angleToPosition(0), DriveTrain.angleToPosition(0),
                DriveTrain.angleToPosition(0), DriveTrain.angleToPosition(0), true);
        // rotDrivePID.disable();
    }

    public static void tick() {
        // this is called roughly 50 times per second

        // SmartDashboard.putBoolean("HasArrived", hasArrived());
        SmartDashboard.putBoolean("TurnCompleted", turnCompleted());
        // SmartDashboard.putNumber("Drive PID Error", DriveTrain.getDriveError());

    }

    public static double getDistanceTravelled() {
        return Math.abs(convertTicksToInches(DriveTrain.getDriveEnc()));
    }

    public static boolean hasArrived() {
        return DriveTrain.hasDriveCompleted(.5); // half inch accuracy
    }

    public static boolean turnCompleted(double allowedError) {
        return Math.abs(RobotGyro.getRelativeAngle() - heading) <= allowedError;
    }

    public static boolean turnCompleted() {
        return turnCompleted(1); // allow 1 degree of error by default
    }

    // public static void resetDriveCurrentBreaker() {
    //     driveCurrentBreaker.reset();
    // }

    // public static boolean isAgainstWall() {
    //     return driveCurrentBreaker.tripped();
    // }

    public static void disable() {
        // setPIDstate(false);
    }

    private static double convertToTicks(double inches) {
        return (double) (inches * Calibration.getDriveTicksPerInch());
    }

    private static double convertTicksToInches(double ticks) {
        return ticks / Calibration.getDriveTicksPerInch();
    }

    public static void showEncoderValues() {
        SmartDashboard.putNumber("Drive Encoder", DriveTrain.getDriveEnc());

        // SmartDashboard.putNumber("Drive PID Error", DriveTrain.getDriveError());
        // SmartDashboard.putNumber("Drive Avg Error",
        // DriveTrain.getAverageDriveError());

        // SmartDashboard.putNumber("Gyro PID Setpoint",
        // rotDrivePID.getSetpoint());
        // SmartDashboard.putNumber("Gyro PID error",
        // round2(rotDrivePID.getError()));

        // SmartDashboard.putBoolean("Has Arrived", hasArrived());

        // SmartDashboard.putNumber("Left Drive Encoder Raw: ",
        // -mainDrive.getLeftEncoderObject().get());
        // SmartDashboard.putNumber("Right Drive Encoder Raw: ",
        // -mainDrive.getRightEncoderObject().get());

        // SmartDashboard.putNumber("Right PID error",
        // rightDrivePID.getError());
        // SmartDashboard.putNumber("Left Drive Encoder Get: ",
        // mainDrive.getLeftEncoderObject().get());
        // SmartDashboard.putNumber("Right Drive Encoder Get: ",
        // mainDrive.getRightEncoderObject().get());
        // SmartDashboard.putNumber("Left Drive Distance: ",
        // leftEncoder.getDistance());
        // SmartDashboard.putNumber("Right Drive Distance: ",
        // rightEncoder.getDistance());
        // SmartDashboard.putNumber("Right Drive Encoder Raw: ",
        // DriveTrain.getDriveEnc());
        // SmartDashboard.putNumber("Right Setpoint: ",
        // rightDrivePID.getSetpoint());

    }
    
    public static void parkingBrake(){
        // DriveTrain.setTurnOrientation(.125, .875, .125, .875, true);
        DriveTrain.setTurnOrientation(.875, .875, .125, .125, true);
    }

    private static Double round2(Double val) {
        // added this back in on 1/15/18
        return new BigDecimal(val.toString()).setScale(2, RoundingMode.HALF_UP).doubleValue();
    }
}
