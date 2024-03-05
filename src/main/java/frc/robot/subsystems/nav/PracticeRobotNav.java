// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import java.util.Optional;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.ExampleSwerveDriveTrain;
import frc.robot.subsystems.nav.Limelight.LimelightOn;
import frc.robot.subsystems.nav.Limelight.LimelightPipeline;
// import frc.robot.subsystems.nav.Limelight.Target;

/** Add your docs here. */
public class PracticeRobotNav extends NavSubsystem {
    public static enum Target {
        SPEAKER,
        AMP, 
    }

    Translation2d position = new Translation2d();
    private NavXGyro gyro;
    private Limelight limelight;
    double yawRotationNudge;
    private SwerveDrivePoseEstimator poseEstimator;
    private ExampleSwerveDriveTrain driveTrain;

    public PracticeRobotNav(ExampleSwerveDriveTrain drive) {
        super();
        driveTrain = drive;

        limelight = new Limelight(new Transform3d(-12,-6.25,2, new Rotation3d(0,Math.toRadians(-45),Math.toRadians(180))));
        limelight.setPipeline(LimelightPipeline.AprilTag);
        limelight.setLED(LimelightOn.Off);
        
        gyro = new NavXGyro(SPI.Port.kMXP);
        
        Optional<Alliance> myAlliance = DriverStation.getAlliance(); 
        if(myAlliance.isPresent() && myAlliance.get() == Alliance.Red){
            useRedTargets();
        } else {
            useBlueTargets();
        }

        poseEstimator = new SwerveDrivePoseEstimator(driveTrain.getKinematics(),
         new Rotation2d(gyro.getGyroAngleInRad()), driveTrain.getOdomotry(), new Pose2d());
    }

    public void zeroYaw(){
        gyro.zeroYaw();
    }

    @Override
    public void reset() {
        gyro.reset();
        poseEstimator.resetPosition(new Rotation2d(gyro.getGyroAngleInRad()), driveTrain.getOdomotry(), new Pose2d());
    }

    @Override
    public double getAngle() {
        return gyro.getAngle();
    }
    private void updateTestPoint(String prefix, Pose2d currentTarget) {
        updateTestPoint(prefix, new Pose3d(currentTarget));
    }

    private void updateTestPoint(String prefix, Pose3d currentTarget) {
        SmartDashboard.putNumber(prefix +" X", currentTarget.getX());
        SmartDashboard.putNumber(prefix + " Y", currentTarget.getY());
        SmartDashboard.putNumber(prefix +" Z", currentTarget.getZ());
        SmartDashboard.putNumber(prefix + " Roll", Math.toDegrees(currentTarget.getRotation().getX()));
        SmartDashboard.putNumber(prefix + " Pitch", Math.toDegrees(currentTarget.getRotation().getY()));
        SmartDashboard.putNumber(prefix + " Yaw", Math.toDegrees(currentTarget.getRotation().getZ()));

    }
    @Override
    public void periodic() {
        updateRobotPosition();

        SmartDashboard.putNumber("Gyro Angle", ((int) (gyro.getAngle() * 1000)) / 1000.0);
        
        computeYawNudge(Target.SPEAKER);
        
        SmartDashboard.putNumber("Yaw Rotation Nudge", yawRotationNudge);
        SmartDashboard.putBoolean("Sees April Tag", limelight.isPoseValid());     
    }

    public void updateRobotPosition() {
        Pose2d limelitePose = limelight.getLimelightPositionInField();
        limelitePose= new Pose2d(limelitePose.getTranslation().times(2.54/100),limelitePose.getRotation());

        poseEstimator.update(new Rotation2d(gyro.getGyroAngleInRad()), driveTrain.getOdomotry());
        if(limelight.isPoseValid()) {
            poseEstimator.addVisionMeasurement(limelitePose, Timer.getFPGATimestamp()-limelight.getLatency());
        }
        
        updateTestPoint("Nav", poseEstimator.getEstimatedPosition());
    }

    public double wrap360To180(double angle){
        double rotations = angle / 360.0;
        rotations -= (int) rotations;
        if (rotations >= 0.5) rotations -= 1.0;
        return rotations * 360.0;
    }

    public void computeYawNudge(Target target) {
        Transform2d currentTarget = getTargetOffset(target);
        updateTestPoint("Nudge",new Pose2d(currentTarget.getTranslation(), currentTarget.getRotation()));
        double goal = 180.0 - currentTarget.getTranslation().getAngle().getDegrees();
        double limit = 0.45;
        double kp = limit/10.0; // limit divided by angle which max power is applied

        yawRotationNudge = kp*wrap360To180(goal);
        yawRotationNudge = Math.min(yawRotationNudge,limit);
        yawRotationNudge = Math.max(yawRotationNudge,-limit);
        yawRotationNudge = -yawRotationNudge;
    }

    public double yawRotationNudge() {
        return yawRotationNudge;
    }
    public boolean isPoseValid() {
        Pose2d pose = poseEstimator.getEstimatedPosition();
        boolean valid = true;
        valid &= pose.getX() >=-2.0;
        valid &= pose.getX() <=8.21055+1;
        valid &= pose.getY() >=-2.0;
        valid &= pose.getY() <=16.54175+1;
        return valid;
    } 
    @Override
    public Translation2d getPosition() {
        return position;
    }
    public Transform2d getTargetOffset(Target target) {
        Pose3d targetPose;
        switch (target) {
            case AMP:
                targetPose = targetPositions.ampPose;
                break;
            case SPEAKER:
                targetPose = targetPositions.speakerPose;
                break;
            default:
                targetPose = new Pose3d();
                break;
        }
        return new Transform2d(getPoseInFieldInches(), targetPose.toPose2d());
    }

    public Pose2d getPoseInFieldInches() {
        return new Pose2d(poseEstimator.getEstimatedPosition().getTranslation().times(100/2.54),poseEstimator.getEstimatedPosition().getRotation());
    }

    public Pose2d getPoseInField() {
        return poseEstimator.getEstimatedPosition();
    }
}