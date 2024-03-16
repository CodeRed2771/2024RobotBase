package frc.robot.subsystems.auto;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    public TrajectoryAuto (CrescendoBot bot) {
        this.bot = bot;
        
        TrajectoryConfig config = new TrajectoryConfig(3,3)
            .setKinematics(bot.drive.getKinematics());

        List<Pose2d> poseList = List.of(new Pose2d(0,0,new Rotation2d(0)),
                 new Pose2d(1,1,new Rotation2d(90)));
        
        firstTrajectory = TrajectoryGenerator.generateTrajectory(
            poseList,
            config);
        
        ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0,
                new TrapezoidProfile.Constraints(
                    Math.PI, Math.PI));
        thetaController.enableContinuousInput(Math.PI, Math.PI);

        PIDController xController = new PIDController(1, 0, 0);
        PIDController yController = new PIDController(1, 0, 0);

        swerveControllerCommand = new SwerveControllerCommand(
            firstTrajectory,
            bot.nav::getPoseInField, 
            bot.drive.getKinematics(),
            xController,
            yController,
            thetaController, 
            bot.drive::setModuleStates);
        
    }

    public Command getAutonomousCommand() {
        return Commands.sequence(
            new InstantCommand(() -> bot.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
            swerveControllerCommand,
            new InstantCommand(() -> bot.drive(0, 0, 0, false)));
    }
}
