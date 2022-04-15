// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  public CANSparkMax intakeRoller;

  private Convoyeur m_conv;

  public CANSparkMax intakeWinch;

  public RelativeEncoder m_encoder;

  public double intakePosOut = 3.0;
  public double intakePosStart = 0;
  public double intakePosIn= 0.5;

  public boolean intakeRequest = false;

  private final NetworkTableEntry m_intakePos;


  /** Creates a new Intake. */
  public Intake(Convoyeur convoyeur) {

    m_conv = convoyeur;
    intakeRoller = new CANSparkMax(Constants.id_intakeRoller, MotorType.kBrushless);
    intakeWinch = new CANSparkMax(Constants.id_intakeWinch, MotorType.kBrushed);
    m_encoder = intakeWinch.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
    m_encoder.setInverted(true);

    var intakeTrainTab = Shuffleboard.getTab("Intake");
    m_intakePos = intakeTrainTab.add("position", 0).getEntry();

  }

  @Override
  public void periodic() {
    m_intakePos.setNumber(getWinchPos());
    // This method will be called once per scheduler run
  }


  //intakes ballons
  public void run() {

    intakeRoller.set(1);
    m_conv.ActivateConvoyeur();
  }

  //reverses the rotation to eject ballon
  public void runReversed(){

    intakeRoller.set(-1);
    m_conv.feedBackward();

  }


  //stops the intake from moving
  public void stop(){
    intakeRoller.set(0);
    m_conv.stop();
  }

  public boolean isRunning(){
    return intakeRoller.get() != 0;
  }

  public boolean isIntaking(){
    return intakeRoller.get() > 0;
  }

  //release the intake with the winch
  public void releaseIntake(){
    intakeRequest = false;
    }
    
//returns current winch position
  public double getWinchPos(){
    return m_encoder.getPosition();
  }

  //retracts the intake
  public void retractIntake(){
    intakeRequest = true;
  }

  public void handleIntakeWinch(){
    if(intakeRequest){
      if(getWinchPos() > intakePosIn){
        intakeWinch.set(1);
      }else{
        intakeWinch.set(0);
      }
    }
    else if(!intakeRequest){
      if(getWinchPos() < intakePosOut){
        intakeWinch.set(-1);
      }else{
        intakeWinch.set(0);
      }
    }
    }
  
  
}
