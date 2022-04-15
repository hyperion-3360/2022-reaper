// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbSequence extends CommandBase {

  enum ClimbState{
    onBar,
    armUp,
    armDown,
    finished
  }

  private Climber m_climber;
  private int m_counter;
  private ClimbState m_currentState = ClimbState.armDown;
  private double m_sens;

  /**
   * Climb to the top.
   * @param climber Climber subsystem instance.
   */
  public ClimbSequence(Climber climber) {   
    m_climber = climber;
    m_sens = climber.rotation;

    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_currentState){
      case onBar:
      
        m_sens *= -1;
        if(m_counter < 3){
          m_counter++;
          m_currentState = ClimbState.armUp;
        } else m_currentState = ClimbState.finished;

      break;

      case armDown:

        m_climber.winch(m_sens);

        if(m_climber.isHooked())
          m_currentState = ClimbState.onBar;

      break;

      case armUp:

        m_climber.winch(m_sens);

        if(m_climber.isHooked())
            m_currentState = ClimbState.onBar;

      break;

      case finished:
        m_climber.setFinalState();
        

      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
