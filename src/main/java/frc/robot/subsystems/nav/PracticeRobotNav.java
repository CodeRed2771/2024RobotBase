// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;


import java.util.Optional;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.ExampleSwerveDriveTrain;
import frc.robot.subsystems.nav.Limelight.LimelightOn;
import frc.robot.subsystems.nav.Limelight.LimelightPipeline;

/** Add your docs here. */
public class PracticeRobotNav extends NavSubsystem {
    public static enum Target {
        SPEAKER,
        AMP, 
    }

    private NavXGyro gyro;
    private Limelight limelight;
    double yawRotationNudge;
    double yawNoteNudge;
    private SwerveDrivePoseEstimator poseEstimator;
    private ExampleSwerveDriveTrain driveTrain;

    public PracticeRobotNav(ExampleSwerveDriveTrain drive) {
        super();
        driveTrain = drive;

        limelight = new Limelight(new Transform3d(-12,-6.25,2, new Rotation3d(0,Math.toRadians(-45),Math.toRadians(180))));
        limelight.setPipeline(LimelightPipeline.NoteTracker);
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
    public void reset(Pose2d init_pose) {
        gyro.reset();
        poseEstimator.resetPosition(new Rotation2d(gyro.getGyroAngleInRad()), driveTrain.getOdomotry(), init_pose);
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
        updateTestPoint(prefix, new Pose3d(pose));
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

        SmartDashboard.putNumber("Gyro Angle", ((int) (gyro.getAngle() * 1000)) / 1000.0);
        
        computeYawNudge(Target.SPEAKER);
        computeNoteNudge();
        
        SmartDashboard.putNumber("Yaw Note Nudge", yawNoteNudge);
        SmartDashboard.putBoolean("Sees April Tag", limelight.isPoseValid());     
    }

    public void updateRobotPosition() {
        Pose2d limelitePose = limelight.getLimelightPositionInField();

        poseEstimator.update(new Rotation2d(gyro.getGyroAngleInRad()), driveTrain.getOdomotry());
        if(limelight.isPoseValid() && gyro.getVelocity3d().getNorm() < 50.0) {
            poseEstimator.addVisionMeasurement(limelitePose, Timer.getFPGATimestamp()-limelight.getLatency());
        }
        
        updateTestPoint("Nav", poseEstimator.getEstimatedPosition());
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
        if(limelight.isNoteValid()) {
            double limit = 0.25;
            double kp = limit/5.0; // limit divided by angle which max power is applied
    
            yawNoteNudge = kp*(0-limelight.horizontalOffset());
            yawNoteNudge = Math.min(yawNoteNudge,limit);
            yawNoteNudge = Math.max(yawNoteNudge,-limit);
            yawNoteNudge = -yawNoteNudge;
        }
    }

    public double yawRotationNudge() {
        return yawRotationNudge;
    }

    public double noteYawNudge() {
        return yawNoteNudge;
    };
    public boolean isNavValid() {
        Pose2d pose = poseEstimator.getEstimatedPosition();
        boolean valid = true;
        valid &= pose.getX() >=-2.0 * 100/2.54;
        valid &= pose.getX() <=(8.21055+1)* 100/2.54;
        valid &= pose.getY() >=-2.0* 100/2.54;
        valid &= pose.getY() <=(16.54175+1)* 100/2.54;
        return valid;
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

}