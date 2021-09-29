/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team696.robot.subsystems.Intake;
import org.team696.robot.subsystems.Spindexer;

public class SpindexerLoading extends CommandBase {
  /**
   * Creates a new SpindexerLoading.
   */
  Spindexer spindexer;
  Intake intake;
  double spindexerPower;
  double intakePower;
  public SpindexerLoading(Spindexer spindexer, Intake intake, double spindexerPower, double intakePower) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.spindexerPower = spindexerPower;
    this.intakePower = intakePower;
    this.spindexer = spindexer;
    this.intake = intake;
    addRequirements(spindexer);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spindexer.spindexerLoadingAntiJam(spindexerPower);
      intake.runIntake(intakePower);
    // System.out.println("running the loading stuff");
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
