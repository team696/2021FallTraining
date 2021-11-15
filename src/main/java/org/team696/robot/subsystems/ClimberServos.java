// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team696.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberServos extends SubsystemBase {
  private Servo climbServo;
  /** Creates a new ClimberServos. */
  public ClimberServos() {
    climbServo = new Servo(4);
  }

  public void UnlockClimber(double angle){
    climbServo.setAngle(angle);
    // climbServo.setPosition(0.5 );   
  }

  public double getServoAngle(){
    return climbServo.getAngle();
  }

  @Override
  public void periodic() {
    // climbServo.setAngle(degrees);
    // This method will be called once per scheduler run
  }
}
