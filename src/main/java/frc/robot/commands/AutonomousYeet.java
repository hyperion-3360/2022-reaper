package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Convoyeur;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class AutonomousYeet extends SequentialCommandGroup {
    
    public AutonomousYeet(
      Intake intake,
      Convoyeur convoyeur,
      DriveTrain driveTrain,
      Shooter shooter,
      Turret turret,
      Vision vision) {

      Trajectory goShoot = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0,new Rotation2d(Math.toRadians(0))),
        List.of(),
        new Pose2d(1.5, 0.0,new Rotation2d(Math.toRadians(0))),
        Constants.autoConfigFastForward);

      Trajectory goAlliance = TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.5, 0.0,new Rotation2d(Math.toRadians(0))),
        List.of(),
        new Pose2d(4.5, 0.0,new Rotation2d(Math.toRadians(0))),
        Constants.autoConfigFastForward);

      Command shootSeq = sequence(
        new InstantCommand(intake::runReversed)
        .andThen(new WaitCommand(0.1))
        .andThen(new InstantCommand(intake::stop))
        .andThen(new WaitCommand(2.0))
        .andThen(new InstantCommand(convoyeur::feed))
        .andThen(new WaitCommand(1.75))
        .andThen(new InstantCommand(shooter::stop))
        .andThen(new InstantCommand(intake::stop)));

      addCommands(
        new InstantCommand(() -> { driveTrain.resetOdometry(goShoot.getInitialPose()); }),

        // Shoot
        race(
          sequence(new CustomRamsete(driveTrain, goShoot), shootSeq),
          new GetShooterReady(shooter, turret, vision)),

        // Yeet
        new CustomRamsete(driveTrain, goAlliance),
        new InstantCommand(intake::run)
      );
    }
}
