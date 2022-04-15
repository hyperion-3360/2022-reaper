/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class SetShooterRpmAuto extends CommandBase {
  private final Shooter m_shooter;
  private final Joystick m_joystick;
  private final Vision m_vision;

  /**
   * Sets the shooter RPM based on the measured distance to the target.
   * @param shooter Shooter subsystem instance.
   * @param joystick Joystick subsystem instance.
   * @param vision Vision subsytem instance.
   */
  public SetShooterRpmAuto(Shooter shooter, Joystick joystick, Vision vision) {
    m_shooter = shooter;
    m_joystick = joystick;
    m_vision = vision;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setSpeedFromDistance(m_vision.getDistance());
    m_joystick.setRumble(RumbleType.kLeftRumble, 0.5);
    
    if (m_shooter.isAtSpeed()) {
      m_vision.setStatusLed(true);
    } else {
      m_vision.setStatusLed(false);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_vision.setStatusLed(false);
    m_joystick.setRumble(RumbleType.kLeftRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
