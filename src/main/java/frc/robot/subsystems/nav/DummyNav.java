// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class DummyNav extends NavSubsystem {
    Pose2d pose = new Pose2d();

    public DummyNav() {
        super();
    }

    @Override
    public void reset(Pose2d init_pose){
        pose = new Pose2d(init_pose.getTranslation(),init_pose.getRotation());
    }
    @Override
    public Pose2d getPoseInField(){
        return new Pose2d(pose.getTranslation(),pose.getRotation());
    }
}