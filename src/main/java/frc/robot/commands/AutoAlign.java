// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class AutoAlign extends CommandBase {
  private final Turret m_turret;
  private final Vision m_vision;

  /**
   * Aligns the turret with the target using the Limelight.
   * @param turret Turret subsystem instance
   * @param vision Vision subsystem instance
   */
  public AutoAlign(Turret turret, Vision vision) {
    m_turret = turret;
    m_vision = vision;
    
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.setVisionMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.aim(m_vision.getTv(), m_vision.getTx());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setRotationSpeed(0);
    m_vision.setVisionMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
