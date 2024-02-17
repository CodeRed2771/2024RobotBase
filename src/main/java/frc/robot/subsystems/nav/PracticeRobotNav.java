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
        Pose3d currentPosition = limelight.getPositionRedAlliance();
        // limelight.pollLimelight();
        SmartDashboard.putNumber("Gyro Angle", ((int) (gyro.getAngle() * 1000)) / 1000.0);
        // updateTestPoint(currentPosition);

        Transform3d currentTarget = limelight.getRedTargetOffset(Target.AMP);
        updateTestPoint(new Pose3d(currentTarget.getTranslation(), currentTarget.getRotation()));
    }

    @Override
    public Translation2d getPosition() {
        return position;
    }
}