package frc.robot.subsystems.auto;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edwpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.CrescendoBot;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class TrajectoryAuto {
    CrescendoBot bot;
    Trajectory firstTrajectory;
    SwerveControllerCommand swerveControllerCommand;

    public TrajectoryAuto(CrescendoBot bot) {
        this.bot = bot;
        
        TrajectoryConfig config = new TrajectoryConfig(3,3)
            .setKinematics(bot.drive.getKinematics());

        // List<Pose2d> poseList = List.of(
        //     new Pose2d(0,0, new Rotation2d(Math.toRadians(0))),
        //     new Pose2d(1,1,new Rotation2d(Math.toRadians(0))),
        //     new Pose2d(2,1,new Rotation2d(Math.toRadians(0)))
        //     );
        
        // firstTrajectory = TrajectoryGenerator.generateTrajectory(
        //     poseList,
        //     config
        // );
        firstTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(10, 10), new Translation2d(20, -10)),
            // End 30 inches straight ahead of where we started, facing forward
            new Pose2d(30, 0, new Rotation2d(0)),
            config);
        
        ProfiledPIDController thetaController = new ProfiledPIDController(.05, 0, 0,
                new TrapezoidProfile.Constraints(
                    Math.PI, Math.PI));
        thetaController.enableContinuousInput(Math.PI, Math.PI);

        PIDController xController = new PIDController(.5, 0, 0);
        PIDController yController = new PIDController(.5, 0, 0);

        swerveControllerCommand = new SwerveControllerCommand(
            firstTrajectory,
            bot.nav::getPoseInField, 
            bot.drive.getKinematics(),
            xController,
            yController,
            thetaController, 
            bot.drive::setModuleStates,
            bot.drive
        );
        
    }

//     public void resetOdometry(Pose2d pose) {
//         bot.drive.getOdomotry().resetPosition(
//             bot.nav.getGyroAngle().toRotation2d(),
//             new SwerveModulePosition[] {
//                 m_frontLeft.getPosition(),
//                 m_frontRight.getPosition(),
//                 m_rearLeft.getPosition(),
//                 m_rearRight.getPosition()
//         },
//         pose);
//   }
    
    public Command getAutonomousCommand() {
        return Commands.sequence(
            new InstantCommand(() -> bot.nav.resetRobotPose(new Pose2d(0,0, new Rotation2d(0)))),
            // new InstantCommand(() -> bot.launcher.fire()),
            swerveControllerCommand,
            new InstantCommand(() -> bot.drive.driveSpeedControl(0, 0, 0, .02)));
    }
}
