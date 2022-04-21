/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {
  /**
   * Creates a new Vision.
   */
  private Solenoid statusLight;
  private NetworkTableEntry distanceCalculated;
  private NetworkTableEntry shooterAligned;

  public Vision() {
    //statusLight = new Solenoid(20, ModuleType.kCTRE, );
    distanceCalculated= Shuffleboard
      .getTab("Pilot View")
      .add("LL distance", 0)
      .getEntry();

    shooterAligned = Shuffleboard
      .getTab("Pilot View")
      .add("turret aligned", false)
      .getEntry();

    var usbCam = CameraServer.startAutomaticCapture();
    usbCam.setFPS(20);
    usbCam.setResolution(176, 144);
  }

  @Override
  public void periodic() {
    distanceCalculated.setDouble(getDistance());
    shooterAligned.setBoolean(isAligned());
  }

  public double getTx(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double getTy(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getTv(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  }
  
  public void setVisionMode(boolean b_state){
    if(b_state){
      //vision mode
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    }else{
      //driver mode
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    }
  }
  
  /**
   * Get the distance to the target.
   * @return distance to target (inches)
   */
  public double getDistance(){
    if(getTv()==1){
      return (100-39) / Math.tan(Math.toRadians(21.3 + getTy()));
    }else{
      return 80;
    }
    
  }

  public void setStatusLed(boolean state){
      statusLight.set(state);
  }

  /**
   * returns true if the turret is aligned
   */
  public boolean isAligned(){
    return getTx() < 2 && getTx() > -2;
  }
}
