package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.libs.CurrentBreaker;

public class DriveAuto {

    private PIDController rotDrivePID;

    private boolean isDriving = false;
    private boolean followingTarget = false;

    private double heading = 0; // keeps track of intended heading - used for driving "straight"
	private double strafeAngle = 0;
	private double GyroAngle = 0;
    private double strafeAngleOriginal = 0;

    // private static CurrentBreaker driveCurrentBreaker;

    private NavXGyro robotGyro = NavXGyro.getInstance();
    private DriveTrain driveTrain = DriveTrain.getInstance();

    public enum DriveSpeed {
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

    public Position currrentPosition; 

    /* Use a singleton design pattern to assist in migrating from ubiquitous static class operations */
    private static class DriveAutoSingleton {
        private static final DriveAuto instance = new DriveAuto();
    }
    public static DriveAuto getInstance(){
        return DriveAutoSingleton.instance;
    }

    private DriveAuto(){        
        rotDrivePID = new PIDController(Calibration.AUTO_ROT_P, Calibration.AUTO_ROT_I, Calibration.AUTO_ROT_D);
    }

    public void init() {

        System.out.println("START OF DRIVEAUTO INIT");

        rotDrivePID.setTolerance(2); // degrees off

        driveTrain.setDriveMMAccel(Calibration.getDT_MM_ACCEL());
        driveTrain.setDriveMMVelocity(Calibration.getDT_MM_VELOCITY());

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

    private void calculateNewDrivePosition(double distance, double angle) {
        double slope;
        if (distance > 0) {
            slope = Math.tan(angle);
        } else {
            slope = Math.tan(-angle);
        }
        double x =1;
        double y = slope*(x+currrentPosition.x) - currrentPosition.y;
    }

    private void calcualteNewTurnPosition(double degrees) {
        currrentPosition.orientation = currrentPosition.orientation + degrees;
    }

	public void driveInches(double inches, double angle, double speedFactor, boolean followTarget) {
		driveInches(inches, angle, speedFactor, followTarget, false);
	}

    public void driveInches(double inches, double angle, double speedFactor, boolean followTarget, boolean fieldCentric) {

        followingTarget = followTarget;

        SmartDashboard.putNumber("DRIVE INCHES", inches);

		if (fieldCentric) {
			GyroAngle = robotGyro.getRelativeAngle();
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

        driveTrain.setDriveMMVelocity((int) (Calibration.getDT_MM_VELOCITY() * speedFactor));

        // angle at which the wheel modules should be turned

        // didnt help - DriveTrain.unReverseModules(); // make sure all "reversed" flags
        // are reset.
        SmartDashboard.putNumber("Strafe Angle:", strafeAngle);
        driveTrain.setAllTurnOrientation(driveTrain.angleToPosition(strafeAngle), true);

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
        driveTrain.addToAllDrivePositions(convertToTicks(inches));
        calculateNewDrivePosition(inches, angle);
    }

    public void driveInches(double inches, double angle, double speedFactor) {
        driveInches(inches, angle, speedFactor, false);
    }

    public void reset() {
        stop();
        driveTrain.resetDriveEncoders();
        rotDrivePID.reset();
        rotDrivePID.setSetpoint(0);
        heading = robotGyro.getRelativeAngle();
        isDriving = false;
    }

    public void stop() {
        stopTurning();
        stopDriving();
    }

    public void stopDriving() {
        isDriving = false;
        driveTrain.stopDriveAndTurnMotors();
    }

    public void stopTurning() {
        // rotDrivePID.setSetpoint(rotDrivePID.calculate(RobotGyro.getAngle())); //
        // changed 1/6/20
        // rotDrivePID.disable();
        // DriveTrain.stopDriveAndTurnMotors();
        driveTrain.stopTurn();
    }

    public void setTurnDegreesToCurrentAngle() {
        // this is necessary so that subsequent turns are relative to the current
        // position. Otherwise they'd always be relative to 0
        rotDrivePID.setSetpoint(robotGyro.getAngle());
    }

    public double degreesToInches(double degrees) {
        double inches = degrees / 4.42; //3.115;
        return inches;
    }

    public void turnToHeading(double desiredHeading, double turnSpeedFactor) {
        // double turnAmount = desiredHeading - RobotGyro.getRelativeAngle();

        double turnAmount = 0;

        if (desiredHeading - robotGyro.getRelativeAngle() > 180)
        {
            turnAmount = (desiredHeading - robotGyro.getRelativeAngle()) - 360;   
        } 
        else if (desiredHeading - robotGyro.getRelativeAngle() < -180)
        {
            turnAmount = (desiredHeading - robotGyro.getRelativeAngle()) + 360; 
        }
        else
        {
            turnAmount = desiredHeading - robotGyro.getRelativeAngle();
        }
        SmartDashboard.putNumber("DESIRED HEADING EQUALS", desiredHeading);
        SmartDashboard.putNumber("ROBOT GYRO RELATIVE", robotGyro.getRelativeAngle());
        SmartDashboard.putNumber("TURN AMOUNT", turnAmount);
        turnDegrees(turnAmount, turnSpeedFactor);
        SmartDashboard.putNumber("NEW ROBOT GYRO ANGLE", robotGyro.getRelativeAngle());
    }

    public void turnDegrees(double degrees, double turnSpeedFactor) {
        // Turns by driving with the modules turned.
        // Use "turnCompleted" method to determine when the turn is done
        // The PID controller for this sends a rotational value to the
        // standard swerve drive method to make the bot rotate

        stopDriving();

        heading += degrees; // this is used later to help us drive straight after rotating

        SmartDashboard.putNumber("TURN DEGREES CALL", degrees);

        driveTrain.setTurnOrientation(driveTrain.angleToPosition(-133.6677), driveTrain.angleToPosition(46.3322),
                driveTrain.angleToPosition(133.6677), driveTrain.angleToPosition(-46.3322), true);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        driveTrain.setDriveMMVelocity((int) (Calibration.getDT_MM_VELOCITY() * turnSpeedFactor));

        double turnInches = degreesToInches(-degrees);
        SmartDashboard.putNumber("Turn Inches", turnInches);
        driveTrain.addToAllDrivePositions(convertToTicks(turnInches));
        calcualteNewTurnPosition(degrees);
    }

    public void continuousDrive(double inches, double maxPower) {
        // setRotationalPowerOutput(maxPower);

        driveTrain.setTurnOrientation(driveTrain.angleToPosition(0), driveTrain.angleToPosition(0),
                driveTrain.angleToPosition(0), driveTrain.angleToPosition(0), true);
        // rotDrivePID.disable();
    }

    public void tick() {
        // this is called roughly 50 times per second

        // SmartDashboard.putBoolean("HasArrived", hasArrived());
        SmartDashboard.putBoolean("TurnCompleted", turnCompleted());
        // SmartDashboard.putNumber("Drive PID Error", DriveTrain.getDriveError());

    }

    public double getDistanceTravelled() {
        return Math.abs(convertTicksToInches(driveTrain.getDriveEnc()));
    }

    public boolean hasArrived() {
        return driveTrain.hasDriveCompleted(.5); // half inch accuracy
    }

    public boolean turnCompleted(double allowedError) {
        return Math.abs(robotGyro.getRelativeAngle() - heading) <= allowedError;
    }

    public boolean turnCompleted() {
        return turnCompleted(1); // allow 1 degree of error by default
    }

    // public void resetDriveCurrentBreaker() {
    //     driveCurrentBreaker.reset();
    // }

    // public boolean isAgainstWall() {
    //     return driveCurrentBreaker.tripped();
    // }

    public void disable() {
        // setPIDstate(false);
    }

    private double convertToTicks(double inches) {
        return (double) (inches * Calibration.getDriveTicksPerInch());
    }

    private double convertTicksToInches(double ticks) {
        return ticks / Calibration.getDriveTicksPerInch();
    }

    public void showEncoderValues() {
        SmartDashboard.putNumber("Drive Encoder", driveTrain.getDriveEnc());

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
    
    public void parkingBrake(){
        // DriveTrain.setTurnOrientation(.125, .875, .125, .875, true);
        driveTrain.setTurnOrientation(.875, .875, .125, .125, true);
    }

    private static Double round2(Double val) {
        // added this back in on 1/15/18
        return new BigDecimal(val.toString()).setScale(2, RoundingMode.HALF_UP).doubleValue();
    }
}
