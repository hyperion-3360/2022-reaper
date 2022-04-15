// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TeleopDriveArcade extends CommandBase {

  private final DriveTrain m_driveTrain;
  private final DoubleSupplier m_driveSupplier;
  private final DoubleSupplier m_turnSupplier;

  //value for the joystick deadzone
  private final static double deadzone = 0.1;

  /**
   * Drive the robot based on a drive and turn axis
   * @param driveTrain DriveTrain subsystem instance.
   * @param drive drive value supplier (-1 to 1).
   * @param turn turn value supplier (-1 to 1).
   */
  public TeleopDriveArcade(DriveTrain driveTrain, DoubleSupplier drive, DoubleSupplier turn) {
    m_driveTrain = driveTrain;
    m_driveSupplier = drive;
    m_turnSupplier = turn;
    
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double d_driveVal = m_driveSupplier.getAsDouble();
    double d_driveCorr = m_turnSupplier.getAsDouble();

    if(d_driveVal < deadzone && d_driveVal > -deadzone) {
      d_driveVal = 0;
    }
    if(d_driveCorr < deadzone && d_driveCorr > -deadzone) {
      d_driveCorr = 0;
    }

    double leftVal = d_driveVal+d_driveCorr;
    double rightVal = d_driveVal-d_driveCorr;
    m_driveTrain.setTank(leftVal, rightVal);
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
