package frc.robot.subsystems.auto;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.CrescendoBot;
import frc.robot.libs.Timer;

public abstract class AutoBaseClass {
    private Timer mAutoTimer; // note that the timer is ticked in isRunning() and hasCompleted()
    private Position mRobotPosition;
    private boolean mIsRunning = false;
    private Direction mDirection;
    protected Pose2d targetPose;
    protected CrescendoBot myRobot;

    public static enum Direction {
        LEFT, RIGHT
    };

    public static enum Position {
        LEFT, CENTER, RIGHT
    };

    public AutoBaseClass(CrescendoBot robot) {
        mAutoTimer = new Timer();

        myRobot = robot;
        targetPose = robot.nav.getPoseInField();    
    }

    public abstract void periodic();

    public void start() {
        mAutoTimer.setStep(0);
        mIsRunning = true;
    }

    public void start(Position robotPosition, Direction direction) {
        mRobotPosition = robotPosition;
        mDirection = direction;
        start();
    }

    public void stop() {
        mIsRunning = false;
    }

    public boolean isRunning() {
        mAutoTimer.tick(); // we need to tick the timer and this is a good place to do it.
        return mIsRunning;
    }

    public boolean hasCompleted() {
        mAutoTimer.tick(); // we need to tick the timer and this is a good place to do it.
        return !mIsRunning;
    }

    public int getCurrentStep() {
        return mAutoTimer.getStep();
    }

    public void setStep(int step) {
        mAutoTimer.setStep(step);
    }

    public double getStepTimeRemainingInSeconds() {
        return mAutoTimer.getTimeRemainingSeconds();
    }

    public double getStepTimeRemainingInMilliSeconds() {
        return mAutoTimer.getTimeRemainingMilliseconds();
    }

    // DRIVE COMMAND!!
    protected void driveFixedPositionOffsetInches(double xInches, double yInches){
        targetPose = targetPose.plus(new Transform2d(xInches,yInches, new Rotation2d()));
        Translation2d nextMove = getDriveTranslation2d();
        myRobot.drive.driveFixedPositionOffsetInches(nextMove.getX(),nextMove.getY());
    }

    // TURN TO HEADING COMMAND!!
    protected void driveFixedRotatePosition(double angle){
        targetPose = targetPose.plus(new Transform2d(0,0,Rotation2d.fromDegrees(-angle)));
        myRobot.drive.driveFixedRotatePosition(getDriveRotation());
    }

    protected Translation2d getDriveTranslation2d()
    {
        return myRobot.nav.getTargetOffset(targetPose).getTranslation();
    }

    protected double getDriveRotation()
    {
        return myRobot.nav.getTargetOffset(targetPose).getRotation().getDegrees();
    }

    // DRIVE COMPLETED COMMAND!!

    // public boolean driveCompleted() {
    //     return DriveAuto.hasArrived();
    // }

    // TURN COMPLETED COMMAND!!

    // public boolean turnCompleted() {
    //     return DriveAuto.turnCompleted();
    // }

    // GENERAL TURN COMMAND

    // public void turnDegrees(double degrees, double maxPower) {
    //   DriveAuto.turnDegrees(degrees, maxPower);
    // }

    public Position robotPosition() {
        return mRobotPosition;
    }

    public void setRobotPosition(Position position) {
        mRobotPosition = position;
    }

    public Direction slideDirection() {
        return mDirection;
    }

    public void advanceStep() {
        mAutoTimer.stopTimerAndAdvanceStep();
    }

    // starts a timer for the time indicated and then immediately advances the
    // stage counter
    // this is typically used when starting a driving maneuver because the next
    // stage would
    // be watching to see when the maneuver was completed.
    public void setTimerAndAdvanceStep(long milliseconds) {
        mAutoTimer.setTimerAndAdvanceStep(milliseconds);
    }

    public void setTimer(long milliseconds) {
        mAutoTimer.setTimer(milliseconds);
    }

    public boolean timeExpired() {
        return mAutoTimer.timeExpired();
    }

    public Optional<Alliance> getAlliance() {
        return DriverStation.getAlliance();
    }

}