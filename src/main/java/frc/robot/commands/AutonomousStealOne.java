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

public class AutonomousStealOne extends SequentialCommandGroup {
    
    public AutonomousStealOne(
      Intake intake,
      Convoyeur convoyeur,
      DriveTrain driveTrain,
      Shooter shooter,
      Turret turret,
      Vision vision) {

      Trajectory getFirstBall = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0,new Rotation2d(Math.toRadians(0))),
        List.of(),
        new Pose2d(1.5, -0.1,new Rotation2d(Math.toRadians(-10))),
        Constants.autoConfigFastForward);

      Trajectory firstReverse = TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.5,-0.1,new Rotation2d(Math.toRadians(-10))),
        List.of(),
        new Pose2d(0, -0.5,new Rotation2d(Math.toRadians(-10))),
        Constants.autoConfigSlowReverse);
    
      Trajectory getFirstOppBall = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,-0.5,new Rotation2d(Math.toRadians(-10))),
        List.of(),
        new Pose2d(1.39, -1.51,new Rotation2d(Math.toRadians(-20))),
        Constants.autoConfigSlowForward);

      Trajectory goHangar = TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.39, -1.51, new Rotation2d(Math.toRadians(-20))), 
        List.of(),
        new Pose2d(3.5, 0.5, new Rotation2d(Math.toRadians(30))),
        Constants.autoConfigFastForward);

      Command shootSeq = sequence(
        new InstantCommand(intake::runReversed)
        .andThen(new WaitCommand(0.1))
        .andThen(new InstantCommand(intake::stop))
        .andThen(new WaitCommand(1.0))
        .andThen(new InstantCommand(convoyeur::feed))
        .andThen(new WaitCommand(1.75))
        .andThen(new InstantCommand(shooter::stop))
        .andThen(new InstantCommand(intake::stop)));

      Command pickupOurSeq = sequence(
        new InstantCommand(intake::run)
        .andThen(new InstantCommand(intake::releaseIntake))
        .andThen(new CustomRamsete(driveTrain, getFirstBall))
        .andThen(new InstantCommand(intake::stop))
      );

      addCommands(
        new InstantCommand(() -> { driveTrain.resetOdometry(getFirstBall.getInitialPose()); }),

        // Pickup our and shoot
        race(
          sequence(pickupOurSeq, shootSeq),
          new GetShooterReady(shooter, turret, vision)),

        // Reverse and pickup their
        new InstantCommand(intake::run),
        new CustomRamsete(driveTrain, firstReverse.concatenate(getFirstOppBall)),
        new InstantCommand(intake::stop),

        // Drive to the hangar and drop the ball
        new CustomRamsete(driveTrain, goHangar),
        new InstantCommand(intake::runReversed),
        new WaitCommand(2.0),
        new InstantCommand(intake::run)
      );
    }
}
