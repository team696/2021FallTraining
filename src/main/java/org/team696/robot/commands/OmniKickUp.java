/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team696.robot.subsystems.Spindexer;

public class OmniKickUp extends CommandBase {
  /**
   * Creates a new OmniKickUp.
   */
  Spindexer spindexer;
  double power;
  public OmniKickUp(Spindexer spindexer, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.spindexer = spindexer;
    this.power = power;
    addRequirements(spindexer);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spindexer.setKickMotor(power);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
