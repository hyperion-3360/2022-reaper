// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Convoyeur extends SubsystemBase {

  //create the talons
  public  CANSparkMax convoyeurBelt;
  public CANSparkMax convoyeurLock;

  private LinearFilter filter;
  private double maxCurrent;
  private double latestCurrent;
  private double currentDelta = 0;

  private static final double kBeltFeedPct = 0.5;
  private static final double kLockPctRatio = -0.75;

  /** Creates a new Convoyeur. */
  public Convoyeur(Turret turret) {
    //motors
    convoyeurBelt = new CANSparkMax(Constants.id_convoyeurBelt, MotorType.kBrushless);
    convoyeurLock = new CANSparkMax(Constants.id_convoyeurLock, MotorType.kBrushless);

    //filter to filter the current drawn by the convoyeurLock
    filter = LinearFilter.singlePoleIIR(0.1, 0.02);
  }

  @Override
  public void periodic() {
    
  }

  public void ActivateConvoyeur(){
    convoyeurBelt.set(0.5);
    convoyeurLock.set(0.5);
  }

  //feeds ballon regardless of the number
  public void feed(){  
    convoyeurBelt.set(kBeltFeedPct);
    convoyeurLock.set(kLockPctRatio * kBeltFeedPct);
  }

  public void feedSlow(){  
    convoyeurBelt.set(0.75 * kBeltFeedPct);
    convoyeurLock.set(kLockPctRatio * kBeltFeedPct);
  }

  public double angleCorrection(double turretAngle){
    double d_corr = 0;
    if(turretAngle > 50 || turretAngle < -50)
    d_corr = -0.2;
    else if(turretAngle > 40 || turretAngle < -40)
    d_corr = -0.3;
    else if(turretAngle > 30 || turretAngle < -30)
    d_corr = -0.35;
    else if(turretAngle > 20 || turretAngle < -20)
    d_corr = -0.3;
    else if(turretAngle > 10 || turretAngle < -10)
    d_corr = -0.20;
    else if(turretAngle > 5 || turretAngle < -5)
    d_corr = -0.20;
    else
    d_corr = 0;   
    return d_corr;
}
  public void feedBackward(){  
    convoyeurBelt.set(-0.4);
    convoyeurLock.set(0.5); 
  }

  //stops the convoyeur
  public void stop(){
    convoyeurBelt.set(0);
    convoyeurLock.set(0);
  }

  //if a ballon has been fed
  public boolean hasFedBallon(){

    return latestCurrent < (maxCurrent-currentDelta);
   
  }

  //reseting the filter and the max current for the next feed
  public void resetFilteringValues(){
    filter.reset();
    maxCurrent = 0;
  }

}
