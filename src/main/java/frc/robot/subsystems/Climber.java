// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
  unlock grimpeur
  dewinch 100%
  maneuvre pilot (recule dedans)
  winch til switch
  unwich til switch
  repeat
*/


package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

// import edu.wpi.first.math.filter.Debouncer;
// import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  //creates the talons
  private TalonFX climberR;
  private TalonFX climberL;

  private DigitalInput switchL;
  private DigitalInput switchR;

  private Servo servoL;
  private Servo servoR;
  

  // private Debouncer debouncerL = new Debouncer(0.1, DebounceType.kFalling);
  // private Debouncer debouncerR = new Debouncer(0.1, DebounceType.kFalling);

  private double maxWinch = 1365000;
  private double minWinch = -1000000;

  public double rotation = -0.8;

  private NetworkTableEntry winchPos;
  private NetworkTableEntry leftHook;
  private NetworkTableEntry rightHook;

  /** Creates a new Climber. */
  public Climber() {
    climberR = new TalonFX(Constants.id_ClimberR);
    climberL = new TalonFX(Constants.id_ClimberL);
    
    servoL = new Servo(Constants.id_climberLeftRelease);
    servoR = new Servo(Constants.id_climberRightRelease);
    
    climberR.follow(climberL); //make the second falcon follow
    climberR.setInverted(true); //invert the rotation

    climberR.setNeutralMode(NeutralMode.Brake);
    climberL.setNeutralMode(NeutralMode.Brake);

    climberL.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
    climberR.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);

    //creates the limit switches
    switchL = new DigitalInput(0); 
    switchR = new DigitalInput(1);

    winchPos = Shuffleboard
    .getTab("Shooter Test")
    .add("winch encoder", 0)
    .getEntry();

    leftHook = Shuffleboard
    .getTab("Pilot View")
    .add("leftHookSwitch", false)
    .getEntry();

    rightHook = Shuffleboard
    .getTab("Pilot View")
    .add("rightHookSwitch", false)
    .getEntry();

    initClimbServos();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    winchPos.setDouble(getWinchPosition());

    leftHook.setBoolean(switchL.get());
    rightHook.setBoolean(switchR.get());
  }

  public void releaseClimb(){
    servoL.set(180);
    servoR.set(180);
  }

  public void initClimbServos(){
    servoL.set(0);
    servoR.set(0);
  }

  // This should be two StartEndCommand, one per button.
  // Call the approriate winch and setFinalState in each.
  // public void Climb(){

  //   System.out.println("isHooked()");

  //   if(RobotContainer.getPilotJoystick().getRawButton(1)) {
  //     winch(rotation);
  //   }
  //   else if(RobotContainer.getPilotJoystick().getRawButton(2)){
  //     winch(-rotation);
  //   }else{
  //     setFinalState();
  //   }
  // }

  //controls the arms with a specified rotation in the sequence
  public void winch(double rotationSpeed){
    climberL.set(ControlMode.PercentOutput, rotationSpeed);
    leftHook.setBoolean(switchL.get());
    rightHook.setBoolean(switchR.get());
  }
  public void climb(){
    if (!isAtMinWinch()) climberL.set(ControlMode.PercentOutput, -Math.abs(RobotContainer.getCopilotJoystick().getRawAxis(1)));
      else stop();
  }
  public void extend(){
    if (!isAtMaxWinch()) climberL.set(ControlMode.PercentOutput, Math.abs(RobotContainer.getCopilotJoystick().getRawAxis(1)));
     else stop();
  }
  public void stop(){
    climberL.set(ControlMode.PercentOutput, 0);
  }

  public boolean isHooked(){
    
    return switchL.get() || switchR.get();

  }
  public boolean isAtMaxWinch(){
    return getWinchPosition() >= maxWinch;
  }
  public boolean isAtMinWinch(){
    return getWinchPosition() <= minWinch;
  }

  public void setFinalState(){
    climberL.set(ControlMode.PercentOutput, 0);
  }

  public double getWinchPosition(){
    return climberL.getSensorCollection().getIntegratedSensorPosition();
  }
}
