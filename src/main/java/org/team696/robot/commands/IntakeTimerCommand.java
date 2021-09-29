/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import org.team696.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeTimerCommand extends CommandBase {
  /**
   * Creates a new IntakeTimerCommand.
   */
  Intake intake;
  int x = 0;
  double timer;
  public IntakeTimerCommand(Intake intake, double timer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.timer = timer;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntake(0.7);
    x++;
    // System.out.println(timer);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // timer = 0;
    intake.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return x>timer;
  }
}
