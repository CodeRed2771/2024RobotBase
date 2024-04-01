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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.BlinkinLED;
import frc.robot.libs.BlinkinLED.LEDColors;
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
    private BlinkinLED nav_led;
    private double distance_travelled = 0;
    private double last_wheel_pos = 0;
    private boolean camera_valid = false;

    private boolean bUseCamera = true;
    public boolean isCameraEnabed() { return bUseCamera; }
    public void setCameraEnable(boolean bUseCamera) { this.bUseCamera = bUseCamera; }
    public void enableCamera() { setCameraEnable(true); }
    public void disableCamera() { setCameraEnable(false); }

    public PracticeRobotNav(Map<String,Integer> wiring,Map<String,Double> calibration,ExampleSwerveDriveTrain drive) {
        super();
        driveTrain = drive;

        limelight_present = calibration.getOrDefault("limelight present", 0.0) > 0.5; 
        if(limelight_present)
        {
            limelight = new LimeLightPoseEstimator("limelight",calibration);
            enableCamera();
        }

        limelight_tracker_present = calibration.getOrDefault("limelight-tracker present",0.0)>0.5;
        if(limelight_tracker_present){
            gamePieceTracker = new LimeLightGamePieceTracker("limelight-tracker",calibration);
        }
        gyro = new NavXGyro(SPI.Port.kMXP);

        poseEstimator = new SwerveDrivePoseEstimator(driveTrain.getKinematics(),
         new Rotation2d(gyro.getGyroAngleInRad()), driveTrain.getOdomotry(), new Pose2d());
        useRedTargets();

        nav_led = new BlinkinLED(wiring.get("nav led"));

        reset();
    }

    public void zeroYaw(){
        gyro.zeroYaw();
        poseEstimator.resetPosition(new Rotation2d(gyro.getGyroAngleInRad()), driveTrain.getOdomotry(), poseEstimator.getEstimatedPosition());
    }

    @Override
    public void reset(Pose2d init_pose) {
        poseEstimator.resetPosition(new Rotation2d(gyro.getGyroAngleInRad()), driveTrain.getOdomotry(), init_pose);
        distance_travelled = 0;

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
    public double getAngleRate(){
        return gyro.yaw_rate();
    }

    @Override
    public Pose2d getPoseInField() {
        return poseEstimator.getEstimatedPosition();
    }

    public Translation2d getVelInRobot(){
        return gyro.getVelocity3d().toTranslation2d();
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

        if(limelight_tracker_present && gamePieceTracker.isTracking())
            nav_led.blink(0.5);
        else
            nav_led.blink(1.0);

        computeYawNudge(Target.SPEAKER);
        computeNoteNudge();
    }

    public void postTelemetry(){
        SmartDashboard.putNumber("Gyro Angle", ((int) (gyro.getAngle() * 1000)) / 1000.0);
        SmartDashboard.putNumber("Yaw Note Nudge", yawNoteNudge);
        updateTestPoint("Nav", poseEstimator.getEstimatedPosition());
        updateTestPoint("Gyro Rates", new Pose3d(gyro.getVelocity3d(),gyro.getRotation()));

        SmartDashboard.putNumber("travel", distance_travelled);

        if(limelight_present)
            SmartDashboard.putBoolean("Sees April Tag", limelight.isTracking());

        if(camera_valid ) nav_led.set(LEDColors.GREEN);
        else if(isNavValid()) nav_led.set(LEDColors.BLUE);
        else nav_led.set(LEDColors.YELLOW);
    }

    public void updateRobotPosition() {

        SwerveModulePosition[] odom = driveTrain.getOdomotry();
        Pose3d camera_pose =  limelight.getEstimatedPosition();
        double cam_error;
        
        distance_travelled += Math.abs(last_wheel_pos - odom[1].distanceMeters); // it really is inches
        last_wheel_pos = odom[1].distanceMeters;
        
        poseEstimator.update(new Rotation2d(gyro.getGyroAngleInRad()), odom);

        if(limelight_present && bUseCamera && gyro.getVelocity3d().getNorm() <= max_camera_speed) {
            camera_pose =  limelight.getEstimatedPosition();
            cam_error = camera_pose.toPose2d().minus(poseEstimator.getEstimatedPosition()).getTranslation().getNorm();

            if(cam_error <= 12.0) distance_travelled = 0.0;
            camera_valid = cam_error <= 24.0 && limelight.isValid();

            limelight.checkUpdatePoseEstimator(poseEstimator);
        }

        if(camera_valid && distance_travelled < 5.0*12.0) camera_valid = false;
    }

    public void computeYawNudge(Target target) {
        Pose2d curPos = getPoseInFieldInches();
        if(isNavValid() && curPos.getTranslation().getX() <= 300.0 ){
            Transform2d currentTarget = getTargetOffset(target);
            updateTestPoint("Nudge",new Pose2d(currentTarget.getTranslation(), currentTarget.getRotation()));
            double goal = 180.0 - currentTarget.getTranslation().getAngle().getDegrees();
            double limit = 0.45;
            double kp = limit/10.0; // limit divided by angle which max power is applied

            yawRotationNudge = kp*MathUtil.inputModulus(goal,-180.0,180.0);
            yawRotationNudge = MathUtil.clamp(yawRotationNudge,-limit, limit);
            yawRotationNudge = -yawRotationNudge;
        } else {
            yawRotationNudge = 0;
        }
    }
    public void computeNoteNudge() {
        gamePieceTracker.update();
        double limit = 0.35;
        double kp = limit/45.0; // limit divided by angle which max power is applied
        yawNoteNudge = kp*(0 - getBearingToNote());
        yawNoteNudge = MathUtil.clamp(yawNoteNudge, -limit, limit);
    }

    public double getBearingToNote(){
        double ang = 0;
        if(limelight_tracker_present && gamePieceTracker.isTracking()) {
            ang = gamePieceTracker.getBearingToTargetDegrees();
        }
        return ang;
    }

    public double yawRotationNudge() {
        return yawRotationNudge;
    }

    public double noteYawNudge() {
        return yawNoteNudge;
    };
    
    public boolean isNavValid() {
        return Crescendo.isValidPosition(poseEstimator.getEstimatedPosition().getTranslation()) && 
               (distance_travelled <= 20.0 * 12.0);
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
        return new Transform2d(getPoseInField(), getTargetPoseField(target).toPose2d());
    }

}