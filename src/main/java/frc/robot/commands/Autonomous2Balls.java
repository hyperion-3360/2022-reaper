// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Convoyeur;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class Autonomous2Balls extends SequentialCommandGroup {

  public Autonomous2Balls(
    DriveTrain driveTrain,
    Intake intake,
    Shooter shooter,
    Turret turret,
    Vision vision,
    Convoyeur convoyeur) {
    
    Trajectory firstTraj = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0,new Rotation2d(0)),
      List.of(),
      new Pose2d(2,0,new Rotation2d(Math.toRadians(0))),
      Constants.autoConfigSlowForward);
  
    Trajectory secondTraj = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2,0, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.5,0,new Rotation2d(0)),
      Constants.autoConfigSlowReverse);
  
    Trajectory fullTraj = firstTraj.concatenate(secondTraj);
    
    addCommands(
      new InstantCommand(() -> { driveTrain.resetOdometry(fullTraj.getInitialPose());}),
      race(
        sequence(
          new InstantCommand(intake::run),
          new InstantCommand(intake::releaseIntake),
          new CustomRamsete(driveTrain, fullTraj),
          new InstantCommand(intake::stop),
          new WaitCommand(3),
          new RunCommand(convoyeur::feed)),
        new GetShooterReady(shooter, turret, vision)
      )
    );
  }
}
