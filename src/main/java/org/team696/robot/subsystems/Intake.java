/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team696.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public CANSparkMax leftIntDepMotor;
  public CANSparkMax rightIntDepMotor;

  public CANPIDController leftIntPID;
  public CANPIDController rightIntPID;

  public WPI_TalonFX intakeRollerMotor;

  /**
   * Creates a new IntakeSubsystem.
   */
    // private CANSparkMax intakeMotor;

  public Intake() {
    leftIntDepMotor = new CANSparkMax(50, MotorType.kBrushless);
    rightIntDepMotor = new CANSparkMax(30, MotorType.kBrushless);
    // rightIntDepMotor.getEncoder().setPosition(0);
    // leftIntDepMotor.getEncoder().setPosition(0);

    leftIntPID = leftIntDepMotor.getPIDController();
    rightIntPID = rightIntDepMotor.getPIDController();


    intakeRollerMotor = new WPI_TalonFX(50);
    // rightEncoder = new CANEncoder(rightIntDepMotor);

    leftIntDepMotor.restoreFactoryDefaults();
    leftIntDepMotor.setInverted(true);
    leftIntPID.setP(Constants.IntakeConstants.leftIntkP);
    leftIntPID.setI(Constants.IntakeConstants.leftIntkI);
    leftIntPID.setD(Constants.IntakeConstants.leftIntkD);
    leftIntPID.setIZone(Constants.IntakeConstants.leftIntIZone);
    leftIntPID.setFF(Constants.IntakeConstants.leftIntFF);
    leftIntPID.setOutputRange(Constants.IntakeConstants.leftIntMin, Constants.IntakeConstants.leftIntMax);

    rightIntDepMotor.restoreFactoryDefaults();
    rightIntDepMotor.setInverted(false);
    // rightEncoder.setPosition(0);
    // rightIntDepMotor.follow(leftIntDepMotor);
    rightIntPID.setP(Constants.IntakeConstants.rightIntkP);
    rightIntPID.setI(Constants.IntakeConstants.rightIntkI);
    rightIntPID.setD(Constants.IntakeConstants.rightIntkD);
    rightIntPID.setIZone(Constants.IntakeConstants.rightIntIZone);
    rightIntPID.setFF(Constants.IntakeConstants.rightIntFF);
    rightIntPID.setOutputRange(Constants.IntakeConstants.rightIntMin, Constants.IntakeConstants.rightIntMax);

    intakeRollerMotor.configFactoryDefault();
    intakeRollerMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    intakeRollerMotor.setSensorPhase(true);
    intakeRollerMotor.setInverted(true);
    intakeRollerMotor.setNeutralMode(NeutralMode.Coast);
    intakeRollerMotor.configAllowableClosedloopError(Constants.IntakeConstants.rollerPIDSlot, 10);
    intakeRollerMotor.configNominalOutputForward(0, Constants.IntakeConstants.intakeTimeout);
    intakeRollerMotor.configNominalOutputReverse(0, Constants.IntakeConstants.intakeTimeout);
    intakeRollerMotor.configPeakOutputForward(1, Constants.IntakeConstants.intakeTimeout);
    intakeRollerMotor.configPeakOutputReverse(-1, Constants.IntakeConstants.intakeTimeout);
    intakeRollerMotor.config_kF(Constants.IntakeConstants.rollerPIDSlot, Constants.IntakeConstants.rollerkF, Constants.IntakeConstants.intakeTimeout);
    intakeRollerMotor.config_kP(Constants.IntakeConstants.rollerPIDSlot, Constants.IntakeConstants.rollerkP, Constants.IntakeConstants.intakeTimeout);
    intakeRollerMotor.config_kI(Constants.IntakeConstants.rollerPIDSlot, Constants.IntakeConstants.rollerkI, Constants.IntakeConstants.intakeTimeout);
    intakeRollerMotor.config_kD(Constants.IntakeConstants.rollerPIDSlot, Constants.IntakeConstants.rollerkD, Constants.IntakeConstants.intakeTimeout);
    
    // intakeMotor = new CANSparkMax(Constants.IntakeConstants.IntakeMotorPort, MotorType.kBrushless);
    // intakeMotor.restoreFactoryDefaults();
    // intakeMotor.setInverted(Constants.IntakeConstants.IntakeInverted);
  }

  public void runIntake(double percent){
    intakeRollerMotor.set(TalonFXControlMode.PercentOutput, percent);
    
  
  }
  public void moveIntake(double intakePosition){
    leftIntPID.setReference(intakePosition, ControlType.kPosition);
    // rightIntPID.setReference(intakePosition, ControlType.kPosition);
  }

  public void MoveRightIntake(double pos){
    rightIntPID.setReference(pos, ControlType.kPosition);
  }




  // public double intakeCurrent(){
  //   return intakeMotor.getOutputCurrent();
  // }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
