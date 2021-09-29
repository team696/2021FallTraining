/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// package org.team696.robot.subsystems;

// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import org.team696.robot.Constants;
// import org.team696.robot.Constants.ClimberConstants;

// public class ClimberServos extends SubsystemBase {
//   private Servo leftClimberServo;
//   private Servo rightClimberServo;

//   /**
//    * Constructor for ServoSubsystem, defines servos
//    */
//   public ClimberServos() {
//     leftClimberServo = new Servo(ClimberConstants.leftClimberServoPort);
//     rightClimberServo = new Servo(ClimberConstants.rightClimberServoPort);

//   }

//   /**
//    * Opens the servos to allow the climber to move down and lift the robot up
//    */
//   public void openServo(){
//     leftClimberServo.set(ClimberConstants.leftServoOpen);
//     rightClimberServo.set(ClimberConstants.rightServoOpen);
//   }


//   public void closeServo(){
//     leftClimberServo.set(ClimberConstants.leftServoClosed);
//     rightClimberServo.set(ClimberConstants.rightServoClosed);
//   }

//   public void moveClimberServos(double angle){
//     // leftClimberServo.setAngle(angle);
//     rightClimberServo.setAngle(angle);
//     System.out.println("running servo to "+angle);

//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
