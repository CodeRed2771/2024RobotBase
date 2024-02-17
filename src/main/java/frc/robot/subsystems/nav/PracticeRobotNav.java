// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.nav.Limelight.LimelightOn;
import frc.robot.subsystems.nav.Limelight.LimelightPipeline;
import frc.robot.subsystems.nav.Limelight.Target;

/** Add your docs here. */
public class PracticeRobotNav extends NavSubsystem {

    Translation2d position = new Translation2d();
    private NavXGyro gyro;
    private Limelight limelight;

    public PracticeRobotNav() {
        super();

        limelight = new Limelight(new Transform3d(14,6.25,4, new Rotation3d(0,Math.toRadians(-30),0)));
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
    
    @Override
    public void periodic() {
        Pose3d currentTarget = limelight.getPositionRedAlliance();
        limelight.pollLimelight();
        SmartDashboard.putNumber("Gyro Angle", ((int) (gyro.getAngle() * 1000)) / 1000.0);
        SmartDashboard.putNumber("Raw Limelight Data", 
            Math.toDegrees(limelight.getPositionRedAlliance().getRotation().getZ()));
        // SmartDashboard.putNumber("Field Pose", limelight.getFieldPose().getRotation().getZ());
        SmartDashboard.putNumber("X Offset Red Aliance April Tags", currentTarget.getX());
        SmartDashboard.putNumber("Y Offset Red Aliance April Tags", currentTarget.getY());
        SmartDashboard.putNumber("Z Offset Red Aliance April Tags", currentTarget.getZ());
        SmartDashboard.putNumber("Roll Offset Red Aliance April Tags", Math.toDegrees(currentTarget.getRotation().getX()));
        SmartDashboard.putNumber("Pitch Offset Red Aliance April Tags", Math.toDegrees(currentTarget.getRotation().getY()));
        SmartDashboard.putNumber("Yaw Offset Red Aliance April Tags", Math.toDegrees(currentTarget.getRotation().getZ()));
    }

    @Override
    public Translation2d getPosition() {
        return position;
    }
}