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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.BlinkinLED;
import frc.robot.libs.BlinkinLED.LEDColors;
import frc.robot.subsystems.drive.ExampleSwerveDriveTrain;
import frc.robot.subsystems.nav.Crescendo.PointsOfInterest;

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
    private SwerveDrivePoseEstimator poseEstimator;
    private ExampleSwerveDriveTrain driveTrain;

    private double allowed_travel_since_pos_sync = 60.0 * 12.0;
    private double camera_update_tolerance = 2.0*12.0;
    private double camera_valid_distance = 2*12.0;
    Field2d report;

    private double max_camera_speed = 50.0;
    private BlinkinLED nav_led;
    private double distance_travelled = 0;
    private double last_wheel_pos = 0;
    private boolean camera_valid = false;

    private boolean bUseCamera = true;
    public boolean isCameraEnabed() { return bUseCamera && limelight_present; }
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

        nav_led = new BlinkinLED(wiring.get("nav led"));
        report = new Field2d();
        SmartDashboard.putData("Field", report);
        
        reset();
    }

    public void zeroYaw(){
        gyro.zeroYaw();
        gyro.setAngleAdjustment( 0 );
        gyro.setAngleAdjustment(-gyro.getAngle());
        poseEstimator.resetPosition(new Rotation2d(-gyro.getGyroAngleInRad()), driveTrain.getOdomotry(), poseEstimator.getEstimatedPosition());
    }

    @Override
    public void reset(Pose2d init_pose) {
        gyro.setAngleAdjustment( 0 );
        gyro.setAngleAdjustment(-gyro.getAngle()-init_pose.getRotation().getDegrees());
        poseEstimator.resetPosition(new Rotation2d(-gyro.getGyroAngleInRad()), driveTrain.getOdomotry(), init_pose);
        distance_travelled = 0;
    }

    public void initializeFieldData(){
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
        if(isCameraEnabed()) limelight.update();
        gamePieceTracker.update();

        updateRobotPosition();

        if(limelight_tracker_present && gamePieceTracker.isTracking())
            nav_led.blink(0.5);
        else
            nav_led.blink(1.0);
    }

    public void postTelemetry(){
        SmartDashboard.putNumber("Gyro Angle", ((int) (gyro.getAngle() * 1000)) / 1000.0);
        updateTestPoint("Nav", poseEstimator.getEstimatedPosition());
        updateTestPoint("Gyro Rates", new Pose3d(gyro.getVelocity3d(),gyro.getRotation()));

        report.setRobotPose(getPoseInField());
        SmartDashboard.putNumber("travel", distance_travelled);

        if(limelight_present)
            SmartDashboard.putBoolean("Sees April Tag", limelight.isTracking());

        if(camera_valid ) nav_led.set(LEDColors.GREEN);
        else if(isNavValid()) nav_led.set(LEDColors.BLUE);
        else nav_led.set(LEDColors.YELLOW);
    }

    public void updateRobotPosition() {

        SwerveModulePosition[] odom = driveTrain.getOdomotry();
        Pose3d camera_pose;
        double cam_error;
        
        distance_travelled += Math.abs(last_wheel_pos - odom[1].distanceMeters); // it really is inches
        last_wheel_pos = odom[1].distanceMeters;
        
        poseEstimator.update(new Rotation2d(-gyro.getGyroAngleInRad()), odom);

        if( isCameraEnabed() && limelight.isValid() && gyro.getVelocity3d().getNorm() <= max_camera_speed) {
            camera_pose =  limelight.getEstimatedPosition();
            cam_error = camera_pose.toPose2d().minus(poseEstimator.getEstimatedPosition()).getTranslation().getNorm();

            limelight.checkUpdatePoseEstimator(poseEstimator);

            if(cam_error <= camera_valid_distance){
                distance_travelled = 0.0;
                camera_valid = true;
            }
        }

        if(camera_valid && distance_travelled >= camera_update_tolerance) camera_valid = false;
    }

    public double getBearingToNote(){
        double ang = 0;
        if(isTrackingNote()) {
            ang = gamePieceTracker.getBearingToTargetDegrees();
        }
        return ang;
    }

    public boolean isTrackingNote()
    {
        return limelight_tracker_present && gamePieceTracker.isTracking();
    }

    public boolean isNavValid() {
        return Crescendo.isValidPosition(poseEstimator.getEstimatedPosition().getTranslation()) && 
               (distance_travelled <= allowed_travel_since_pos_sync);
    }

    public double getBearingToTarget(Pose3d target){
        return getBearingToTarget(target.getTranslation().toTranslation2d());
    }

    public double getBearingToTarget(Translation2d target){
        Transform2d offset = getTargetOffset(target);
        return offset.getTranslation().getAngle().getDegrees();
    }

    public Transform2d getTargetOffset(Translation2d target){
        return getTargetOffset(new Pose2d(target, new Rotation2d()));
    }

    public Transform2d getTargetOffset(Pose2d target){
        return new Transform2d(getPoseInField(), target);
    }
}