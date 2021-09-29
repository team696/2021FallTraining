/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import org.team696.robot.subsystems.Spindexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ContinuousShoot extends CommandBase {
  /**
   * Creates a new ContinuousShoot.
   */
  Spindexer spindexer;
  boolean kickUpState;
  double drumSpeed;
  int timer;
  public ContinuousShoot(Spindexer spindexer, double drumSpeed, boolean kickUpState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.spindexer = spindexer;
    this.kickUpState = kickUpState;
    this.drumSpeed = drumSpeed;
    addRequirements(spindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer++;
    if(timer>6){
    spindexer.spindexerLoadingAntiJam(drumSpeed);

    }
    spindexer.setKickMotor(1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spindexer.setKickMotor(0);
    timer = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
