/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterHood extends SubsystemBase {
  /**
   * Creates a new ShooterHood.
   */
  
  private Servo leftShooterHoodServo;
  private Servo rightShooterHoodServo;

  public ShooterHood() {
    leftShooterHoodServo = new Servo(2);
    rightShooterHoodServo = new Servo(3);

  }

  public void moveServoAngle(double angle) {
    // if (shooterHoodServo.getAngle() > Constants.ShooterConstants.shooterHoodAngleMinLimit
    //     || shooterHoodServo.getAngle() < Constants.ShooterConstants.shooterHoodAngleMaxLimit) {
      leftShooterHoodServo.setAngle(angle);
      rightShooterHoodServo.setAngle(180-angle);;
    // } else {

    // }

  }

  public void moveServoPosition(double position) {
    leftShooterHoodServo.setPosition(position);
    rightShooterHoodServo.setPosition(1-position);
  }


    /**
   * Gets last commanded hood servo angle
   * @return servo angle
   */
  public double servoAngle(){
    return leftShooterHoodServo.getAngle();
    // return leftShooterHoodServo.getPosition();
  }

  
  public double servoAngleToShootAngle(double servoAngle){
    return -0.65828*(servoAngle)+59.3891;
    //70 servo is 14 shoot
    //10 servo is 53 shoot
  }

  public double shootAngleToServoAngle(double shootAngle){
    return -1.5256*(shootAngle)+90.9203;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double testTargetAngle = SmartDashboard.getNumber("Target Angle", 30);
  }
}
