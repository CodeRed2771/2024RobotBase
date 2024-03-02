// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.ExampleSwerveDriveTrain;
import frc.robot.subsystems.nav.Limelight.LimelightOn;
import frc.robot.subsystems.nav.Limelight.LimelightPipeline;
import frc.robot.subsystems.nav.Limelight.Target;

/** Add your docs here. */
public class PracticeRobotNav extends NavSubsystem {

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
        useRedTargets();
    }

    public void zeroYaw(){
        gyro.zeroYaw();
    }

    @Override
    public void reset() {
        gyro.reset();
    }

    @Override
    public double getAngle() {
        return gyro.getAngle();
    }
    private void updateTestPoint(Pose2d currentTarget) {
        updateTestPoint(new Pose3d(currentTarget));
    }

    private void updateTestPoint(Pose3d currentTarget) {
        SmartDashboard.putNumber("X Test Point", currentTarget.getX());
        SmartDashboard.putNumber("Y Test Point", currentTarget.getY());
        SmartDashboard.putNumber("Z Test Point", currentTarget.getZ());
        SmartDashboard.putNumber("Roll Test Point", Math.toDegrees(currentTarget.getRotation().getX()));
        SmartDashboard.putNumber("Pitch Test Point", Math.toDegrees(currentTarget.getRotation().getY()));
        SmartDashboard.putNumber("Yaw Test Point", Math.toDegrees(currentTarget.getRotation().getZ()));

    }
    @Override
    public void periodic() {

        updateRobotPosition();

        // Pose3d currentPosition = limelight.getPositioninField();
        // limelight.pollLimelight();
        SmartDashboard.putNumber("Gyro Angle", ((int) (gyro.getAngle() * 1000)) / 1000.0);
        // updateTestPoint(currentPosition);
        if(isSeeingAprilTags()) {
            computeYawNudge();
        } else {
            yawRotationNudge = 0;
        }
        
        SmartDashboard.putNumber("Yaw Rotation Nudge", yawRotationNudge);
        SmartDashboard.putBoolean("Sees April Tag", isSeeingAprilTags());     
    }

    public void updateRobotPosition() {
        Pose2d limelitePose = limelight.getLimelightPositionInField();
        limelitePose= new Pose2d(limelitePose.getTranslation().times(2.54/100),limelitePose.getRotation());

        poseEstimator.update(new Rotation2d(gyro.getGyroAngleInRad()), driveTrain.getOdomotry());
        if(isSeeingAprilTags()) {
            poseEstimator.addVisionMeasurement(limelitePose, Timer.getFPGATimestamp()-limelight.getLatency());
        }

        updateTestPoint(poseEstimator.getEstimatedPosition());
    }

    public void computeYawNudge() {
        Transform2d currentTarget = getTargetOffset(Target.AMP);
        // updateTestPoint(new Pose2d(currentTarget.getTranslation(), currentTarget.getRotation()));


        double limit = 0.25;
        double kp = limit/5.0; // limit divided by angle which max power is applied

        yawRotationNudge = kp*(0-currentTarget.getRotation().getDegrees());
        yawRotationNudge = Math.min(yawRotationNudge,limit);
        yawRotationNudge = Math.max(yawRotationNudge,-limit);
        yawRotationNudge = -yawRotationNudge;
    }

    public double yawRotationNudge() {
        return yawRotationNudge;
    }
    public boolean isSeeingAprilTags() {
        return limelight.seesSomething() && limelight.getPipeline() == LimelightPipeline.AprilTag;
    } 
    @Override
    public Translation2d getPosition() {
        return position;
    }
    public Transform2d getTargetOffset(Target target) {
        Pose3d targetPose;
        useRedTargets();
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
        return new Transform2d(getPoseInField(), targetPose.toPose2d());
    }

    public Pose2d getPoseInField() {
        return poseEstimator.getEstimatedPosition();
    }
}