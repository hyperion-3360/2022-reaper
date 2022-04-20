// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Convoyeur;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class ShooterSequence extends CommandBase {

  enum ShooterState{
    aim,
    feed
  }

  private Shooter m_shooter;
  private Convoyeur m_convoyeur;
  private Turret m_turret;
  private Vision m_vision;

  private ShooterState m_currentState = ShooterState.aim;

  /** Creates a new ShooterSequence. */
  public ShooterSequence(Shooter shooter, Convoyeur convoyeur, Turret turret, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(convoyeur);
    addRequirements(turret);

    m_shooter = shooter;
    m_convoyeur = convoyeur;
    m_turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentState = ShooterState.aim;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    switch(m_currentState){
      case aim:
        m_turret.aim(m_vision.getTv(), m_vision.getTx(), false);
        m_shooter.setSpeedDistance(m_vision.getDistance());
        m_convoyeur.stop();
        
        if(m_vision.isAligned() && m_shooter.isAtSpeed()){
          m_currentState = ShooterState.feed;
          m_convoyeur.resetFilteringValues();
        }

      break;

      case feed:
        m_turret.aim(m_vision.getTv(), m_vision.getTx(), false);
        m_shooter.setSpeedDistance(m_vision.getDistance());
        m_convoyeur.feed();

        if(m_convoyeur.hasFedBallon()){
          m_currentState = ShooterState.aim;
          m_convoyeur.stop();
        }

      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
