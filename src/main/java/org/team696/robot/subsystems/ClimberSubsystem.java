/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// package org.team696.robot.subsystems;

// import com.revrobotics.CANEncoder;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.FaultID;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.EncoderType;
// import com.revrobotics.Rev2mDistanceSensor;
// import com.revrobotics.Rev2mDistanceSensor.Port;
// import com.revrobotics.Rev2mDistanceSensor.Unit;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import org.team696.robot.Constants;
// import org.team696.robot.Constants.ClimberConstants;
// import org.team696.robot.Constants.OperatorConstants;
// import org.team696.robot.RobotContainer;

// public class ClimberSubsystem extends SubsystemBase {
//   /**
//    * Creates a new ClimberSubsystem.
//    */

//   public CANSparkMax leftClimberMotor;
//   private CANSparkMax rightClimberMotor;

//   private CANEncoder leftClimbEncoder;
//   private CANEncoder rightClimbEncoder;

//   // private ElevatorFeedforward climberFeedForward;
//   // private Rev2mDistanceSensor climberLatchSensor;

//   public double climberTargetPower = 0;

//   private enum OperatorClimberStates {
//     REACH, LIFT, OFF, BOTH_ON
//   }

//   private OperatorClimberStates climberStates = OperatorClimberStates.OFF;

//   public ClimberSubsystem() {
//     leftClimberMotor = new CANSparkMax(Constants.ClimberConstants.leftClimberPort, MotorType.kBrushed);

//     // leftClimbEncoder.setPositionConversionFactor(Constants.ClimberConstants.positionConversionFactor);
//     // leftClimbEncoder.setVelocityConversionFactor(Constants.ClimberConstants.velocityConversionFactor);

//     rightClimberMotor = new CANSparkMax(Constants.ClimberConstants.rightClimberPort, MotorType.kBrushed);
//     configRightClimber();
//     configLeftClimber();

//     leftClimbEncoder = new CANEncoder(leftClimberMotor, EncoderType.kNoSensor, 10);
//     rightClimbEncoder = new CANEncoder(rightClimberMotor, EncoderType.kNoSensor, 10);
//     // rightClimbEncoder.setPositionConversionFactor(Constants.ClimberConstants.positionConversionFactor);
//     // rightClimbEncoder.setVelocityConversionFactor(Constants.ClimberConstants.velocityConversionFactor);

//     // climberFeedForward = new ElevatorFeedforward(Constants.ClimberConstants.kS,
//     // Constants.ClimberConstants.kG, Constants.ClimberConstants.kV,
//     // Constants.ClimberConstants.kA);

//     // climberLatchSensor = new Rev2mDistanceSensor(Port.kOnboard);
//   }

//   /**
//    * Initializes climber subsystem.
//    */
//   public void initialize() {
//     leftClimbEncoder.setPosition(0);
//     rightClimbEncoder.setPosition(0);
//   }

//   // public double distanceRange() {
//   //   return climberLatchSensor.getRange(Unit.kMillimeters);
//   // }

//   public double leftEncoderPosition() {
//     return leftClimbEncoder.getPosition();
//   }

//   public double rightEncoderPosition() {
//     return rightClimbEncoder.getPosition();
//   }

//   public double leftEncoderVelocity() {
//     return leftClimbEncoder.getVelocity();
//   }

//   public double rightEncoderVelocity() {
//     return rightClimbEncoder.getVelocity();
//   }

//   public double leftCurrent() {
//     return leftClimberMotor.getOutputCurrent();
//   }

//   public double rightCurrent() {
//     return rightClimberMotor.getOutputCurrent();
//   }

//   public double climberAppliedPower(){
//     return leftClimberMotor.getAppliedOutput();
//   }

//   /**
//    * Sets climberTargetPower based on climber state.
//    */
//   public void updateClimberPower() {
//     switch (climberStates) {
//     case REACH:
//       climberTargetPower = ClimberConstants.climberReachTargetPower;
//       break;

//     case LIFT:
//       climberTargetPower = ClimberConstants.climberLiftTargetPower;
//       break;

//     case OFF:
//       climberTargetPower = 0;
//       break;

//     case BOTH_ON:
//       climberTargetPower = 0;
//       break;

//     default:
//       climberTargetPower = 0;
//       break;

//     }

//   }
//   /**
//    * Sets open-loop climber power
//    */
//   public void setClimberPower() {
//     leftClimberMotor.set(climberTargetPower);
//     System.out.println(climberTargetPower);
//     // rightClimberMotor.set(climberTargetPower);

//   }

//   /**
//    * Sets open-loop climber power.
//    * @param power Throttle value
//    */
//   public void setClimberPower(double power) {
//     leftClimberMotor.set(power);
//     System.out.println(power);
//     // rightClimberMotor.set(power);

//   }

//   // might add sparkmax built in pid velocity control TBD with testing
//   public void setClimberVelocity() {

//   }

//   @Override
//   public void periodic() {

//     if (leftClimberMotor.getFault(FaultID.kHasReset)) {
//       configLeftClimber();
//     }
//     if (rightClimberMotor.getFault(FaultID.kHasReset)) {
//       configRightClimber();
//     }

//     if (RobotContainer.operatorPanel.getRawButton(OperatorConstants.climberDownButton)
//         && !RobotContainer.operatorPanel.getRawButton(OperatorConstants.climberUpButton)) {
//       climberStates = OperatorClimberStates.REACH;
//     } else if (RobotContainer.operatorPanel.getRawButton(OperatorConstants.climberUpButton)
//         && !RobotContainer.operatorPanel.getRawButton(OperatorConstants.climberDownButton)) {
//       climberStates = OperatorClimberStates.LIFT;
//     } else if (!RobotContainer.operatorPanel.getRawButton(OperatorConstants.climberDownButton)
//         && !RobotContainer.operatorPanel.getRawButton(OperatorConstants.climberUpButton)) {
//       climberStates = OperatorClimberStates.OFF;
//     } else if (RobotContainer.operatorPanel.getRawButton(OperatorConstants.climberDownButton)
//         && RobotContainer.operatorPanel.getRawButton(OperatorConstants.climberUpButton)) {
//       climberStates = OperatorClimberStates.BOTH_ON;
//     } else {
//       climberStates = OperatorClimberStates.OFF;
//     }

//     updateClimberPower();

//   }

//   /**
//    * Sets up left climber motor.
//    */
//   public void configLeftClimber() {
//     leftClimberMotor.restoreFactoryDefaults();
//     leftClimberMotor.setInverted(ClimberConstants.leftClimberInverted);
//     leftClimberMotor.setIdleMode(IdleMode.kBrake);
//     leftClimberMotor.setMotorType(MotorType.kBrushed);
//     // leftClimberMotor.setSmartCurrentLimit(80);

//     // leftClimberMotor.setOpenLoopRampRate(ClimberConstants.openLoopRampRate);
//     // leftClimberMotor.follow(rightClimberMotor, ClimberConstants.leftClimberInverted);
//   }

//   /**
//    * Sets up right climber motor. 
//    */
//   public void configRightClimber() {
//     rightClimberMotor.restoreFactoryDefaults();
//     rightClimberMotor.setInverted(ClimberConstants.leftClimberInverted);

//     rightClimberMotor.setIdleMode(IdleMode.kBrake);
//     rightClimberMotor.setMotorType(MotorType.kBrushed);
//     // rightClimberMotor.setOpenLoopRampRate(ClimberConstants.openLoopRampRate);

//     rightClimberMotor.follow(leftClimberMotor, false);

//   }
// }
