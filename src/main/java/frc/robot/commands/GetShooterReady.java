// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class GetShooterReady extends CommandBase {
  private final Shooter m_shooter;
  private final Turret m_turret;
  private final Vision m_vision;

  /**
   * Accelerate the shooter to the correct RPM based on the distance to the target.
   * @param shooter Shooter subsystem instance.
   * @param turret Turret subsystem instance.
   * @param vision Vision subsytem instance.
   */
  public GetShooterReady(Shooter shooter, Turret turret, Vision vision) {
    m_shooter = shooter;
    m_turret = turret;
    m_vision = vision;

    //addRequirements(m_shooter);
    //addRequirements(m_turret);

   /* m_rpmSetpoint = Shuffleboard
      .getTab("Shooter Test")
      .add("Target RPM", 0)
      .getEntry();
      */
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.setVisionMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_turret.aim(m_vision.getTv(), m_vision.getTx());
    m_shooter.setSpeedFromDistance(m_vision.getDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_turret.setRotationSpeed(0);
    m_vision.setVisionMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
