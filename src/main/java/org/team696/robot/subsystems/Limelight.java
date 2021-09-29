/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.subsystems;

import org.team696.robot.Constants.LimelightConstants;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /**
   * Creates a new LimeLightSybsystem.
   */


  NetworkTableEntry camTran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran");

  public static double[] camTranArray = {0.0,0.0,0.0,0.0,0.0,0.0};

  // public double[] camTranArray = new double[6];

  // public LimelightSubsystem() {

  // }

  // public void camTran() {
  //   camTranArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran")
  //       .getDoubleArray(camTranArray);
  // }

  /**
   * 
   * @param lightMode 1 is off, 3 is on
   */
  public static void setLights(int lightMode){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(lightMode);

  }

  public void updateCamTran(){
    camTranArray = camTran.getDoubleArray(camTranArray);
  }

  public boolean is3dModePipelineOn(){
    return (getPipeline()==LimelightConstants.camTranPipeline);
  }


  public static double distanceFromTarget(){
    return camTranArray[2];
  }

  

  public double tx() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public static double ty() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double tyradian() {
    return Math.toRadians(ty());
  }

  public double tvert() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);
  }

  public double thor() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
  }

  public double tlong() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tlong").getDouble(0);
  }

  public double tshort() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tshort").getDouble(0);
  }

  public double pixels() {
    return thor() * tvert();
  }

  public void pipeline(int x) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setDouble(x);
  }

  public static double getPipeline() {

    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(0);
  }

  public double thorDistance() {

    return 11.6 / (2 * Math.tan(Math.toRadians((0.17 / 2) * thor())));
  }

  public double tvertDistance() {

    return 1.85 / (2 * Math.tan(Math.toRadians((0.17 / 2) * tvert())));

  }

  public static double angleDistance(){
    return (98.25-24.5)/Math.tan(Math.toRadians(27.5+ty()));
  }

  /**
   * 
   * @return 1 if has target, 0 if no target
   */
  public static double hasTarget(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
 
  }

  public boolean crosshairOnTarget(){
    if(hasTarget()==1){
      //need to adjust this threshold. maybe can change depending on distance from target
      return tx()<1;
    }
    else{
      return false;
    }
  }

    /**
   * 
   * @return Returns index of the TrajectoryTable array to look for based on the
   *         distance the limelight returns. If it returns 0 then it means there
   *         is no target and the limelight is not configured in 3d Mode. In this
   *         scenario it selects the first row of the array with an angle of 0 and
   *         a speed of 0.
   *
   *         May need to adjust the default angle value
   */
  public static int indexForDistance() {
    // need to add code to prevent out of index of the array
    // if statement for if there is a target and we are in 3d mode
    if (Math.abs(angleDistance())>119&&Math.abs(angleDistance())<(400)) {
      return (int)Math.abs(angleDistance()) - 119;
    } else
      return 0;
  }

  @Override
  public void periodic() {

    // pipeline(5);
    updateCamTran();

  }
}