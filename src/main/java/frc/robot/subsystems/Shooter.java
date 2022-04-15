// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private CANSparkMax shooter1;
  private CANSparkMax shooter2;
  
  private SparkMaxPIDController m_pidShooter;
  public double dashRPM, kP, kI, kD, kIz, kFF, kS, kMaxOutput, kMinOutput, maxRPM, lastSetpoint, vNom;

  private final double kDst2RPM_m = 10;
  private final double kDst2RPM_b = 1775;

  /** Creates a new Shooter. */
  public Shooter() {
    kP = 0.00005; 
    kI = 0.0;
    kIz = 0.0;
    kD = 0.0; 
    kS = 0.0165195;
    kFF = 0.000223861;
    vNom = 10.0;

    kMaxOutput = -1; 
    kMinOutput = 1;

    maxRPM = 5500;
    dashRPM = 5000;

    shooter1 = new CANSparkMax(Constants.id_Shooter1, MotorType.kBrushless);
    shooter2 = new CANSparkMax(Constants.id_Shooter2, MotorType.kBrushless);
    
    shooter1.restoreFactoryDefaults();
    shooter2.restoreFactoryDefaults();

    shooter1.enableVoltageCompensation(vNom);
    shooter2.enableVoltageCompensation(vNom);
    shooter1.setIdleMode(IdleMode.kCoast);
    shooter2.setIdleMode(IdleMode.kCoast);
    
    m_pidShooter = shooter1.getPIDController();
    shooter2.follow(shooter1, true);

    m_pidShooter.setP(kP);
    m_pidShooter.setI(kI);
    m_pidShooter.setD(kD);
    //m_pidShooter.setIZone(kIz);
    m_pidShooter.setFF(kFF);
    m_pidShooter.setOutputRange(kMinOutput, kMaxOutput);

    shooter1.burnFlash();
    shooter2.burnFlash();
  }

  //@Override
  public void periodic() {
    SmartDashboard.putNumber("SHTrpm", shooter1.getEncoder().getVelocity());
  }

  public void setSpeed(double rpm){
    m_pidShooter.setReference(-rpm, ControlType.kVelocity, 0, Math.signum(-rpm) * kS, ArbFFUnits.kPercentOut);
    SmartDashboard.putNumber("SHTrpm_cmd", rpm);
    
  }

  public void setAutoPreSpin(){
    setSpeed(2000);
  }
    
  

  /**
   * Set the shooter RPM based on the distance to the target
   * @param distance distance to target (inches)
   */
  public void setSpeedFromDistance(double distanceInches){
    double d_speed = (distanceInches * kDst2RPM_m) + kDst2RPM_b;
    lastSetpoint = d_speed;
    d_speed = Math.abs(d_speed);
    setSpeed(d_speed);
    //setSpeed(reqRPM.getDouble(2000.0));
  }

  public void setShooterVoltage(double input){
    //m_pidShooter.setReference(input, ControlType.kVoltage);
  }

  public void setDashboardSpeed(){
    //System.out.println("setSpeed = " + SmartDashboard.getNumber("RPM", 0));
   // m_pidShooter.setReference(SmartDashboard.getNumber("RPM", 0), ControlType.kVelocity);
  }

  public boolean isAtSpeed(){
    return lastSetpoint - shooter1.getEncoder().getVelocity() > -100 && 
           lastSetpoint - shooter1.getEncoder().getVelocity() < 100;
  }



  public void shoot(){

  
  }

  /**
   * 
   * @param distance en metres
   */
  public void setSpeedDistance(double distance){

  }

  /**
   * Get the current speed of the shooter.
   * @return shooter wheel RPM
   */
  public double getSpeed() {
    return -shooter1.getEncoder().getVelocity();
  }


  public void stop() {
    shooter1.set(0);

  }
}
