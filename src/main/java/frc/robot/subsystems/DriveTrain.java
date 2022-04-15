// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class DriveTrain extends SubsystemBase {

  //creates the motors
  private TalonFX driveL1 = new TalonFX(Constants.id_DriveL1);
  private TalonFX driveL2 = new TalonFX(Constants.id_DriveL2);
  private TalonFX driveR1 = new TalonFX(Constants.id_DriveR1);
  private TalonFX driveR2 = new TalonFX(Constants.id_DriveR2);



  //creates the gyro
  private final Gyro m_gyro = new ADXRS450_Gyro(); 

  private final DifferentialDriveOdometry m_odometry;

  private final double kP = 0.16266;//0.16266;

  private final double kWheelDiam = 0.100838;
  private final double kDtRatio = 7.86;

  // distance entries to dashboard
  

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    //left is inverted
    driveL1.setInverted(true);
    driveL2.setInverted(true);
    //set PID values
    driveL1.config_kP(0, kP);
    driveR1.config_kP(0, kP);
    //acceleration ramp
    driveL1.configOpenloopRamp(0.5);
    driveR1.configOpenloopRamp(0.5);

    //makes the second motor follow the first one
    driveL2.follow(driveL1);
    driveR2.follow(driveR1);

    driveL2.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
    driveR2.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
    driveL1.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 50);
    driveR1.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 50);

    resetEncoders();
    zeroHeading();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d().unaryMinus());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(m_gyro.getRotation2d().unaryMinus(), -getLeftEncoderPositionMeters(), getRightEncoderPositionMeters());
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }
 /**
  * 
  * @return wheel velocities in meters per second
  */
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocityMetersPerSec(), getRightEncoderVelocityMetersPerSec());
  }
  /**
   * Set left and right side.
   * @param left left side percentage (-1 to 1)
   * @param right right side percentage (-1 to 1)
   */
  public void setTank(double left, double right) {
    driveL1.set(ControlMode.PercentOutput, left);
    driveR1.set(ControlMode.PercentOutput, right);
  }

  public void stop(){
    driveL1.set(ControlMode.PercentOutput, 0);
    driveR1.set(ControlMode.PercentOutput, 0);
  }

  public void setDriveVelocity(double leftSpeed, double rightSpeed){

    double leftVelTicks = leftSpeed * kDtRatio * 2048 / (kWheelDiam * Math.PI);
    double rightVelTicks = rightSpeed * kDtRatio * 2048 / (kWheelDiam * Math.PI);

    driveL1.set(ControlMode.Velocity, leftVelTicks / 10.0); //DemandType.ArbitraryFeedForward, leftFeedPercent / 10.0);
    driveR1.set(ControlMode.Velocity, rightVelTicks / 10.0); //DemandType.ArbitraryFeedForward, rightFeedPercent / 10.0);

  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    zeroHeading();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d().unaryMinus());
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  
  public void resetEncoders() {
    driveL1.getSensorCollection().setIntegratedSensorPosition(0, 0);
    driveR1.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }
  public double getLeftEncoderPositionMeters(){
    return driveL1.getSensorCollection().getIntegratedSensorPosition()/(kDtRatio*2048)*(kWheelDiam*Math.PI);
  }
  public double getRightEncoderPositionMeters(){
    return driveR1.getSensorCollection().getIntegratedSensorPosition()/(kDtRatio*2048)*(kWheelDiam*Math.PI);
  }
  public double getLeftEncoderVelocityMetersPerSec(){
      return (driveL1.getSensorCollection().getIntegratedSensorVelocity()/(kDtRatio*2048)*(kWheelDiam*Math.PI))*10;
    }
  public double getRightEncoderVelocityMetersPerSec(){
      return (driveR1.getSensorCollection().getIntegratedSensorVelocity()/(kDtRatio*2048)*(kWheelDiam*Math.PI))*10;
  }
    
  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  
  public double getAverageEncoderDistance() {
    return (driveL1.getSensorCollection().getIntegratedSensorPosition() + 
            driveR1.getSensorCollection().getIntegratedSensorPosition()) / 2.0;
  }
   
  /**
   * Zeroes the heading of the robot.
   */
  
  public void zeroHeading() {
    m_gyro.reset();
  }
  
  //set the motors to coast
  public void setCoastMode(){
    
    driveL1.setNeutralMode(NeutralMode.Coast);
    driveR1.setNeutralMode(NeutralMode.Coast);
  }
    
  
    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    
    public double getHeading() {
      return m_gyro.getRotation2d().getDegrees();
    }
  
    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    
    public double getTurnRate() {
      return -m_gyro.getRate();
    }
    
    

    public void DriveWithEncoders(double distance, double angleOffset){
      double leftSpeed = OutputFromDist(distance, driveL1.getSensorCollection().getIntegratedSensorPosition());
      double rightSpeed = -OutputFromDist(distance, -driveR1.getSensorCollection().getIntegratedSensorPosition());
      if(leftSpeed > 0.4){
        leftSpeed = 0.4;
      }
      if(rightSpeed > 0.4){
      rightSpeed = 0.4;
      }
      if(rightSpeed < -0.4){
        rightSpeed = -0.4;
      }
      if(leftSpeed < -0.4){
        leftSpeed = -0.4;
      }
      leftSpeed += (angleOffset/90);
      rightSpeed += (angleOffset/90);
      driveL1.set(ControlMode.PercentOutput, leftSpeed);
      driveR1.set(ControlMode.PercentOutput, rightSpeed);
    }

    public double OutputFromDist(double setpoint, double currentDist){
      return setpoint - EncToInches(currentDist) / 18;
    }
    public double EncToInches(double encTicks){
      return encTicks/1300;
    }
    public boolean isAtSetPoint(double setpoint){
      return setpoint - EncToInches(getAverageEncoderDistance()) < 7.5 &&
             setpoint - EncToInches(getAverageEncoderDistance()) > -7.5;
    }
    public void setRampRate(double time){
      driveL1.configOpenloopRamp(time);
      driveR1.configOpenloopRamp(time);
    }
    public void setMotors(double lOutput, double rOutput){
      driveL1.set(ControlMode.PercentOutput, -lOutput);
      driveR1.set(ControlMode.PercentOutput, rOutput);
    }

    public void driveReverse(){
      driveL1.set(ControlMode.PercentOutput, 0.2);
      driveR1.set(ControlMode.PercentOutput, 0.2);
    }

}
