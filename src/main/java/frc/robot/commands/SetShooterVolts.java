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

public class SetShooterVolts extends CommandBase {
  private final Shooter m_shooter;
  private final Vision m_vision;
  private final Joystick m_joystick;

  private final double m_volts;
  
  /**
   * Apply a given voltage to the shooter motors.
   * @param shooter Shooter subsystem instance.
   * @param vision Vision subsystem instance.
   * @param joystick Joystick subsystem instance.
   * @param volts voltage to apply (V).
   */
  public SetShooterVolts(Shooter shooter, Vision vision, Joystick joystick, double volts) {
    m_shooter = shooter;
    m_vision = vision;
    m_joystick = joystick;
    m_volts = volts;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setShooterVoltage(m_volts);

    if(m_volts == 0){
      m_joystick.setRumble(RumbleType.kLeftRumble, 0);
      m_vision.setStatusLed(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
