// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Leds extends SubsystemBase {

  private Solenoid ledR;
  private Solenoid ledL;
  private Solenoid ledF;
  private Solenoid ledB;

  /** Creates a new Leds. */
  public Leds() {

    ledR = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.id_ledR);
    ledL = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.id_ledL);
    ledF = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.id_ledF);
    ledB = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.id_ledB);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void ledsOn(){
    ledR.set(true);
    ledL.set(true);
    ledF.set(true);
    ledB.set(true);
  }
}
