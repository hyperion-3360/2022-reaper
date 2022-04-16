package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class CustomRamsete extends RamseteCommand
{
    public CustomRamsete(DriveTrain driveTrain, Trajectory trajectory) {
        super(
            trajectory,
            driveTrain::getPose,
            new RamseteController(),
            Constants.driveKinematics,
            driveTrain::setDriveVelocity,
            driveTrain);
    }
}