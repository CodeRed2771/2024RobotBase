// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;


import java.util.Map;
import java.util.Optional;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.ExampleSwerveDriveTrain;

/** Add your docs here. */
public class PracticeRobotNav extends NavSubsystem {
    public enum Target {
        SPEAKER,
        AMP, 
    }

    private NavXGyro gyro;
    private LimeLightPoseEstimator limelight;
    private LimeLightGamePieceTracker gamePieceTracker;
    private boolean limelight_present = false;
    private boolean limelight_tracker_present = false;
    double yawRotationNudge;
    double yawNoteNudge;
    private SwerveDrivePoseEstimator poseEstimator;
    private ExampleSwerveDriveTrain driveTrain;

    private double max_camera_speed = 50.0;

    private boolean bUseCamera = true;
    public boolean isCameraEnabed() { return bUseCamera; }
    public void setCameraEnable(boolean bUseCamera) { this.bUseCamera = bUseCamera; }
    public void enableCamera() { setCameraEnable(true); }
    public void disableCamera() { setCameraEnable(false); }

    public PracticeRobotNav(Map<String,Double> calibration,ExampleSwerveDriveTrain drive) {
        super();
        driveTrain = drive;

        limelight_present = calibration.getOrDefault("limelight present", 0.0) > 0.5; 
        if(limelight_present)
        {
            limelight = new LimeLightPoseEstimator("limelight",calibration);
            enableCamera();
        }

        limelight_tracker_present = calibration.getOrDefault("limelight_tracker present",0.0)>0.5;
        if(limelight_tracker_present){
            gamePieceTracker = new LimeLightGamePieceTracker("limelight_tracker",calibration);
        }
        gyro = new NavXGyro(SPI.Port.kMXP);

        poseEstimator = new SwerveDrivePoseEstimator(driveTrain.getKinematics(),
         new Rotation2d(gyro.getGyroAngleInRad()), driveTrain.getOdomotry(), new Pose2d());
        useRedTargets();

         reset();
    }

    public void zeroYaw(){
        gyro.zeroYaw();
    }

    @Override
    public void reset(Pose2d init_pose) {
        gyro.reset();
        poseEstimator.resetPosition(new Rotation2d(gyro.getGyroAngleInRad()), driveTrain.getOdomotry(), init_pose);

        if(limelight_present){
            Optional<Alliance> myAlliance = DriverStation.getAlliance(); 
            if(myAlliance.isPresent() && myAlliance.get() == Alliance.Red){
                limelight.useRedTargets();
            } else {
                limelight.useBlueTargets();
            }
        }
    }

    /* TODO: Remove this after poseEstimator is working well */
    @Override
    public double getAngle() {
        return gyro.getAngle();
    }

    @Override
    public Pose2d getPoseInField() {
        return poseEstimator.getEstimatedPosition();
    }

    private void updateTestPoint(String prefix, Pose2d pose) {
        SmartDashboard.putNumber(prefix + " X", pose.getX());
        SmartDashboard.putNumber(prefix + " Y", pose.getY());
        SmartDashboard.putNumber(prefix + " Yaw", pose.getRotation().getDegrees());
    }

    private void updateTestPoint(String prefix, Pose3d pose) {
        SmartDashboard.putNumber(prefix + " X", pose.getX());
        SmartDashboard.putNumber(prefix + " Y", pose.getY());
        SmartDashboard.putNumber(prefix + " Z", pose.getZ());
        SmartDashboard.putNumber(prefix + " Roll", Math.toDegrees(pose.getRotation().getX()));
        SmartDashboard.putNumber(prefix + " Pitch", Math.toDegrees(pose.getRotation().getY()));
        SmartDashboard.putNumber(prefix + " Yaw", Math.toDegrees(pose.getRotation().getZ()));
    }
    @Override
    public void periodic() {
        updateRobotPosition();

        computeYawNudge(Target.SPEAKER);
        computeNoteNudge();
    }

    public void postTelemetry(){
        SmartDashboard.putNumber("Gyro Angle", ((int) (gyro.getAngle() * 1000)) / 1000.0);
        SmartDashboard.putNumber("Yaw Note Nudge", yawNoteNudge);
        updateTestPoint("Nav", poseEstimator.getEstimatedPosition());
        updateTestPoint("Gyro Rates", new Pose3d(gyro.getVelocity3d(),gyro.getRotation()));

        if(limelight_present)
            SmartDashboard.putBoolean("Sees April Tag", limelight.isTracking());

    }

    public void updateRobotPosition() {

        poseEstimator.update(new Rotation2d(gyro.getGyroAngleInRad()), driveTrain.getOdomotry());

        if(limelight_present && bUseCamera && gyro.getVelocity3d().getNorm() <= max_camera_speed) {
            limelight.checkUpdatePoseEstimator(poseEstimator);
        }
    }

    public void resetRobotPose(Pose2d pose) {
        poseEstimator.resetPosition(new Rotation2d(gyro.getGyroAngleInRad()), driveTrain.getOdomotry(), pose);
    }

    public void computeYawNudge(Target target) {
            Transform2d currentTarget = getTargetOffset(target);
            updateTestPoint("Nudge",new Pose2d(currentTarget.getTranslation(), currentTarget.getRotation()));
            double goal = 180.0 - currentTarget.getTranslation().getAngle().getDegrees();
            double limit = 0.45;
            double kp = limit/10.0; // limit divided by angle which max power is applied

            yawRotationNudge = kp*MathUtil.inputModulus(goal,-180.0,180.0);
            yawRotationNudge = MathUtil.clamp(yawRotationNudge,-limit, limit);
            yawRotationNudge = -yawRotationNudge;
        
    }
    public void computeNoteNudge() {
        if(limelight_tracker_present && gamePieceTracker.isTracking()) {
            double limit = 0.25;
            double kp = limit/5.0; // limit divided by angle which max power is applied
    
            yawNoteNudge = kp*(0- gamePieceTracker.getBearingToTargetDegrees());
            yawNoteNudge = Math.min(yawNoteNudge,limit);
            yawNoteNudge = Math.max(yawNoteNudge,-limit);
            yawNoteNudge = -yawNoteNudge;
        } else {
            yawNoteNudge = 0.0;
        }
    }

    public double yawRotationNudge() {
        return yawRotationNudge;
    }

    public double noteYawNudge() {
        return yawNoteNudge;
    };
    
    public boolean isNavValid() {
        return Crescendo.isValidPosition(poseEstimator.getEstimatedPosition().getTranslation());
    }

    public Pose3d getTargetPoseField(Target target){
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
        return targetPose;
    }
    public Transform2d getTargetOffset(Target target) {
        return new Transform2d(getPoseInFieldInches(), getTargetPoseField(target).toPose2d());
    }

    public Rotation3d getGyroAngle() {
        return gyro.getRotation();
    }
}