// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team696.robot.Constants.ClimberConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  WPI_TalonFX rightClimberMotor;
  WPI_TalonFX leftClimberMotor;

  /** Creates a new Climber. */
  public Climber() {

    leftClimberMotor = new WPI_TalonFX(30);
    rightClimberMotor = new WPI_TalonFX(20);

    leftClimberMotor.configFactoryDefault();
    leftClimberMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, ClimberConstants.climberTimeout);
    leftClimberMotor.setSensorPhase(true);;
    leftClimberMotor.setInverted(true);;
    leftClimberMotor.configNominalOutputForward(0, ClimberConstants.climberTimeout);
    leftClimberMotor.configNominalOutputReverse(0, ClimberConstants.climberTimeout);
    leftClimberMotor.configPeakOutputForward(1, ClimberConstants.climberTimeout);
    leftClimberMotor.configPeakOutputReverse(-1, ClimberConstants.climberTimeout);
    leftClimberMotor.configAllowableClosedloopError(ClimberConstants.climberPIDSlot, 1, ClimberConstants.climberTimeout);
    leftClimberMotor.config_kF(ClimberConstants.climberPIDSlot, ClimberConstants.climberkF, ClimberConstants.climberTimeout);
    leftClimberMotor.config_kP(ClimberConstants.climberPIDSlot, ClimberConstants.climberkP, ClimberConstants.climberTimeout);
    leftClimberMotor.config_kI(ClimberConstants.climberPIDSlot, ClimberConstants.climberkI, ClimberConstants.climberTimeout);
    leftClimberMotor.config_kD(ClimberConstants.climberPIDSlot, ClimberConstants.climberkD, ClimberConstants.climberTimeout);

    rightClimberMotor.configFactoryDefault();
    rightClimberMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, ClimberConstants.climberTimeout);
    rightClimberMotor.setSensorPhase(true);
    rightClimberMotor.setInverted(true);
    rightClimberMotor.follow(leftClimberMotor);
    rightClimberMotor.configNominalOutputForward(0, ClimberConstants.climberTimeout);
    rightClimberMotor.configNominalOutputReverse(0, ClimberConstants.climberTimeout);
    rightClimberMotor.configPeakOutputForward(1, ClimberConstants.climberTimeout);
    rightClimberMotor.configPeakOutputReverse(-1, ClimberConstants.climberTimeout);
    rightClimberMotor.configAllowableClosedloopError(ClimberConstants.climberPIDSlot, 1, ClimberConstants.climberTimeout);
    rightClimberMotor.config_kF(ClimberConstants.climberPIDSlot, ClimberConstants.climberkF, ClimberConstants.climberTimeout);
    rightClimberMotor.config_kP(ClimberConstants.climberPIDSlot, ClimberConstants.climberkP, ClimberConstants.climberTimeout);
    rightClimberMotor.config_kI(ClimberConstants.climberPIDSlot, ClimberConstants.climberkI, ClimberConstants.climberTimeout);
    rightClimberMotor.config_kD(ClimberConstants.climberPIDSlot, ClimberConstants.climberkD, ClimberConstants.climberTimeout);

  }

  public void RunClimber(double per){
    leftClimberMotor.set(TalonFXControlMode.PercentOutput, per);
    // rightClimberMotor.set(TalonFXControlMode.PercentOutput, per);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
