// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class DummyNav extends NavSubsystem {
    Translation2d position = new Translation2d();
    Rotation2d orientation = new Rotation2d();

    public DummyNav() {
        super();
    }

    @Override
    public double getAngle(){
        return orientation.getDegrees();
    }

    @Override
    public Translation2d getPosition(){
        return position;
    }
}