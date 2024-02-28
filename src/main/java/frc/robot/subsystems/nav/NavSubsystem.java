// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class NavSubsystem extends SubsystemBase {
  protected NavSubsystem() {
    super();
  }

  public abstract class fieldPositions {
    private fieldPositions() {}
    Pose3d ampPose;
    Pose3d speakerPose;
    Pose3d[] trapPose = new Pose3d[4];
    Pose3d feederStation;
    // Pose3d[] speakerAngledPose = new Pose3d[2];
  }

  public class blueFieldPositions extends fieldPositions {
    public blueFieldPositions() {
      
        feederStation = new Pose3d(613.05,26.46,53.38, new Rotation3d(0,0,Math.toRadians(120)));
        speakerPose = new Pose3d(652.73,218.42,40.13,new Rotation3d(0,0,Math.toRadians(180)));
        ampPose = new Pose3d(578.77,323.00,41.38,new Rotation3d(0,0,Math.toRadians(270)));
        trapPose[1] = new Pose3d(468.69,146.19,43.00,new Rotation3d(0,0,Math.toRadians(300)));
        trapPose[2] = new Pose3d(468.69,177.10,43.00,new Rotation3d(0,0,Math.toRadians(60)));
        trapPose[3] = new Pose3d(441.74,161.62,43.00,new Rotation3d(0,0,Math.toRadians(180)));
    }
  }
  public class redFieldPositions extends fieldPositions {
    public redFieldPositions() {
      ampPose = new Pose3d(72.5,0,0,new Rotation3d(0,0,Math.toRadians(90)));
      speakerPose = new Pose3d(34.50,218.42,0,new Rotation3d(0,0,0));
      trapPose[1] = new Pose3d(209.48,161.62,43.00,new Rotation3d(0,0,0));
      trapPose[2] = new Pose3d(182.73,177.10,43.00,new Rotation3d(0,0,Math.toRadians(120)));
      trapPose[3] = new Pose3d(182.73,146.19,43.00,new Rotation3d(0,0,Math.toRadians(240)));
      feederStation = new Pose3d(38.17,26.46,53.38,new Rotation3d(0,0,Math.toRadians(60)));
      // speakerAngledPose[1] = new Pose3d(34.50,218.42,0,new Rotation3d(0,0,30));
      // speakerAngledPose[2] = new Pose3d(34.50,218.42,0,new Rotation3d(0,0,-30));
    }
  }
  protected fieldPositions targetPositions;

  public void useBlueTargets() {
    targetPositions = new blueFieldPositions();
  }
  public void useRedTargets() {
    targetPositions = new redFieldPositions();
  }


  public void reset() {
  }

  public abstract Translation2d getPosition();
  public double getAngle() {
    return 0;
  }
  
}
