/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team696.robot.Constants.TurretConstants;
import org.team696.robot.Robot;

public class TurretSubsystem extends SubsystemBase {
  /**
   * Creates a new TurretSubsystem.
   */
  private WPI_TalonSRX turretMotor;
  private PIDController turretController;

  private double turretSpeed;

  public TurretSubsystem() {
    turretMotor = new WPI_TalonSRX(TurretConstants.turretMotorPort);
    turretMotor.configFactoryDefault();
    // turretMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    turretMotor.setSensorPhase(TurretConstants.turretMotorSensorPhase);
    turretMotor.setInverted(TurretConstants.turretMotorInverted);
    turretMotor.setNeutralMode(NeutralMode.Brake);

    turretMotor.configNominalOutputForward(0);
    turretMotor.configNominalOutputReverse(0);
    turretMotor.configPeakOutputForward(TurretConstants.peakOutput);
    turretMotor.configPeakOutputReverse(-TurretConstants.peakOutput);

    turretMotor.configForwardSoftLimitThreshold(TurretConstants.forwardSoftLimit);
    turretMotor.configReverseSoftLimitThreshold(TurretConstants.reverseSoftLimit);
    turretMotor.configForwardSoftLimitEnable(false);
    turretMotor.configReverseSoftLimitEnable(false);

    turretController = new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);

    turretController.setTolerance(TurretConstants.positionTolerance);
  }

  /**
   * Gets tx value from Limelight.
   * 
   * @return
   */
  public double getTx() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  /**
   * Runs turret in open-loop.
   * 
   * @param speed Output throttle
   */
  public void openLoop(double speed) {
    turretMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Checks if turret is on target.
   * 
   * @return True if on target (position error within tolerance)
   */
  public boolean onTarget() {
    return (Math.abs(turretController.getPositionError()) < TurretConstants.positionTolerance) && Limelight.hasTarget()==1;
  }

  public void moveToTarget() {
    turretSpeed = turretController.calculate(getTx(), 0);
    turretMotor.set(ControlMode.PercentOutput, turretSpeed);
  }

  /**
   * @param position moves turret to a set position in talon encoder units
   */
  // todo: add some kind of degrees to encoder unit conversion
  public void moveToPosition(double position) {
    turretMotor.set(ControlMode.Position, position);
  }

  /**
   * 
   * @return Gets potentiometer positon based off the talon's analog input mode
   */
  // public double getPotPosition() {
  //   return turretMotor.getSelectedSensorPosition();
  // }

  /**
   * 
   * @return Gets supply current 
   */
  public double getCurrent() {
    return turretMotor.getSupplyCurrent();
  }

  public double getVelocity() {
    return turretMotor.getSelectedSensorVelocity();
  }

  public double getRPM() {
    return 0;
  }

  // public double setpoint() {
  //   return turretController.getSetpoint();
  // }

  // public double getError() {
  //   return turretController.getPositionError();
  // }

  // public void resetEncoder() {
  //   turretMotor.setSelectedSensorPosition(0);
  // }

  // public double encoderToDegrees(double encoder) {
  //   return encoder / 1.1366666666666666666 - 581 + 130;
  // }

  public void setPIDSlot(int slot) {
    turretMotor.selectProfileSlot(slot, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("degrees", encoderToDegrees(getPotPosition()));
    // SmartDashboard.putNumber("pot position", getPotPosition());
    SmartDashboard.putNumber("tx", getTx());
    Robot.m_robotContainer.setLimelightCaptureLED((Limelight.hasTarget()==1));
    Robot.m_robotContainer.setLimelightLockLED(Math.abs(turretController.getPositionError()) < TurretConstants.positionTolerance);

  }
}
