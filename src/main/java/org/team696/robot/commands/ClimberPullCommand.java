// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team696.robot.commands;

import org.team696.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberPullCommand extends CommandBase {
  Climber climber;
  double percent;


  /** Creates a new ClimberPullCommand. */
  public ClimberPullCommand(Climber climber, double percent ) {
    this.climber = climber;
    this.percent = percent;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.RunClimber(percent);
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
