// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team696.robot.commands;

import org.team696.robot.Constants.IntakeConstants;
import org.team696.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;

public class IntakeDownCommand extends CommandBase {
  Intake intake;
  /** Creates a new IntakeDownCommand. */
  public IntakeDownCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.MoveRightIntake(IntakeConstants.intakeDownPosition);
    intake.moveIntake(IntakeConstants.intakeDownPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
