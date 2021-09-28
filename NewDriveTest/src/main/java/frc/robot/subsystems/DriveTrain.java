// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.CancelablePrintJob;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  DifferentialDrive diffDrive;
  SpeedControllerGroup leftSide;
  SpeedControllerGroup rightSide;
  CANSparkMax leftFront;
  CANSparkMax leftBack;
  CANSparkMax rightFront;
  CANSparkMax rightBack;

  
  public DriveTrain(int leftFrontPort, int leftBackPort, int rightFrontPort, int rightBackPort) {
    leftFront = new CANSparkMax(leftFrontPort, MotorType.kBrushless);
    leftBack = new CANSparkMax(leftBackPort, MotorType.kBrushless);
    rightBack = new CANSparkMax(rightBackPort, MotorType.kBrushless);
    rightFront = new CANSparkMax(rightFrontPort, MotorType.kBrushless);

    leftSide = new SpeedControllerGroup(leftFront, leftBack);
    rightSide = new SpeedControllerGroup(rightFront, rightBack);

    diffDrive = new DifferentialDrive(leftSide, rightSide);
  }

  public void robotDrive(double leftSpeed, double rightSpeed){
    diffDrive.tankDrive(leftSpeed, rightSpeed);
    diffDrive.setDeadband(0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
