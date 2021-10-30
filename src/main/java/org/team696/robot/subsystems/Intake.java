/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team696.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  /**
   * Creates a new IntakeSubsystem.
   */
    private CANSparkMax intakeMotor;

  public Intake() {
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.IntakeMotorPort, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(Constants.IntakeConstants.IntakeInverted);
  }

  public void runIntake(double power){
    intakeMotor.set(power);
    // System.out.println("intaking");
  }


  public double intakeCurrent(){
    return intakeMotor.getOutputCurrent();
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
