/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team696.robot.Constants.IntakeConstants;
import org.team696.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  /**
   * Creates a new IntakeCom
   * mand.
   */
  private Intake intake;

  public IntakeCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntake(IntakeConstants.rollerVelocity);
    intake.moveIntake(IntakeConstants.intakeDownPosition);
    intake.MoveRightIntake(IntakeConstants.intakeDownPosition);
    // spindexer.spindexerLoadingAntiJam(power, current);
    // System.out.println("loading");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
