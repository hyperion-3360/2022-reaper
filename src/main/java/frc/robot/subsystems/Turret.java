// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Turret extends SubsystemBase {

  //-------DO NOT CHAMGE THESE VALUES, THEY ARE THE TURRET'S LIMITS---------////
  private double minAngle = -290/5; //turret minimum angle in encoder ticks
  private double maxAngle = 100/5;  //turret maximum angle in encoder ticks
  
  private double softZoneSize = 10;

  private double rejectOffset = 10/5;

  private double center = 0;

  private double climbAngle = -280/5;

  private CANSparkMax turretTurner; //the motor
  private ColorSensorV3 colorSensor; //the color sensor

  private double angleTolerance = 25; //
  private double scanSpeed = 0.75; //speed at which the scan is performed
  private double rotationSpeed = 0; 
  private double maxTurretPercent = 0.75; //max rotation speed of the motor: 0.75 = 75%

  private Alliance alliance;
  private double redThreshold = 0.2;
  private double blueThreshold = 0.2;

  private NetworkTableEntry turretCurrent;

  /** Creates a new Turret. */
  public Turret() {
    turretTurner = new CANSparkMax(Constants.id_TurretTurner, MotorType.kBrushless);

    turretTurner.setIdleMode(IdleMode.kBrake);
    turretTurner.setOpenLoopRampRate(0.125);

    turretCurrent = Shuffleboard
      .getTab("Shooter Test")
      .add("turretCurrent", 0)
      .getEntry();

    colorSensor = new ColorSensorV3(Port.kMXP);
    colorSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes20bit, 
                                     ColorSensorMeasurementRate.kColorRate2000ms, 
                                     GainFactor.kGain3x);
    alliance = DriverStation.getAlliance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //System.out.println(turretTurner.getEncoder().getPosition() + "encoder pos");
    //System.out.println(m_vision.getTv() + "vision TV");
    turretCurrent.setDouble(turretTurner.getOutputCurrent());
  }

  public boolean minAngleReached(){
    return getTurretAngle() < minAngle;
  }
  public boolean maxAngleReached(){
    return getTurretAngle() > maxAngle;
  }

  public void aimManual(){
    aim(1, 40*RobotContainer.getCopilotJoystick().getRawAxis(4));
  }

  /**
   * Aim using the vision outputs.
   * @param tv target is valid (1)
   * @param tx target horizontal angle position in frame (-29 to 29 deg)
   */
  public void aim(double tv, double tx){

    //if the limelight detects a valid target
    if(tv == 1){

      //if there'S a valid target the rotation speed is the 
      //limelight's output
      rotationSpeed = GetAlignmentOutputs(tx); 

    }else{

      //if there's no valid target the rotation speed is the scan speed
      rotationSpeed = scanSpeed;

    }

    //if the turret's angle is greater than it's lower limit and 
    //in the rotation speed is negative
    if((getTurretAngle() > minAngle && rotationSpeed < 0)){

      if(getTurretAngle() < (minAngle + softZoneSize)){
        rotationSpeed = rotationSpeed/5;
      }

    }else if(getTurretAngle() < maxAngle && rotationSpeed  > 0){

      if(getTurretAngle() > (maxAngle - softZoneSize)){
        rotationSpeed = rotationSpeed/5;
      }

    }else{//if there's no target the scan speed is reversed and the rotation speed is set to 0
      rotationSpeed = 0;
      scanSpeed = -scanSpeed;
    }
    turretTurner.set(rotationSpeed);
  }

  /**
   * Sets the speed of the motor
   * @param speed
   */
  public void setRotationSpeed(double speed){
    turretTurner.set(speed);
  }

  /**
   * Sets the turret in the climbing position
   */
  public void setTurretClimb(){
    aimWithValue(climbAngle);
  }

  /**
   * Aim with a specified value in encoder ticks
   * @param angle
   */
  public void aimWithValue(double angle){

    double angleOffset = 0;
    angleOffset = (getTurretAngle() - angle)*6;
    aim(1, angleOffset);
  }

  /**
   * 
   * @return the encoder's position
   */
  public double getTurretAngle(){
    return turretTurner.getEncoder().getPosition();
  }

  /**
   * returns the speed of the turret according to it's offset
   * @param d_offset the limelight's crossair offset
   * @return motor speed
   */
  public double GetAlignmentOutputs(double d_offset){

    double value = -d_offset/20; //normalizing the value

    if(d_offset < angleTolerance && d_offset > -angleTolerance)
      value = -d_offset/100;

    //if the normalized value is too big it's set to the max speed
    if (value > maxTurretPercent){

      value = maxTurretPercent;

    }else if(value < -maxTurretPercent){

      value = -maxTurretPercent;
    }

    //returns the value
    return value;
  }

  /**
   * Sets the turret to it's starting postion
   */
  public void toCenter() {

    aimWithValue(center);

  }

  /**
   * Checks if the ballon that is about to get shoot is the right color according to the allance color
   * The color sensor return a red, green and blue value. The bigger it is the more likely it's detecting that value
   * Only chnage the threshold if required, but they were really good at where they were...
   * @param Tv if there's a valid target
   * @param Tx target offset
   */
  public void checkColor(double Tv, double Tx){

    //temporary variable to do a comparaison
    Alliance tempAlliance = Alliance.Invalid; 

    //if the color meets the red threshold
    if(colorSensor.getColor().red > redThreshold){

      //the temporsary alliance is set to red
      tempAlliance = Alliance.Red;
    }
    
    //if the color meets the blue threshold
    if(colorSensor.getColor().blue > blueThreshold){

      //the temporary alliance is set to blue
      tempAlliance = Alliance.Blue;
    }

    //if the alliance is the right one the turret aim normally
    if(tempAlliance == alliance)
      aim(Tv, Tx);

    //otherwise it aims with an offset to not shoot in the goal
    else aimWithValue(Tx - rejectOffset);
  }

}
