/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team696.robot.subsystems.Spindexer;

public class OmniKickUpTimer extends CommandBase {
  /**
   * Creates a new OmniKickUp.
   */
  Spindexer spindexer;
  boolean state;
  int x = 0;
  double timer;

  public OmniKickUpTimer(Spindexer spindexer, boolean state, double timer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.spindexer = spindexer;
    this.state = state;
  
    this.timer = timer;
    addRequirements(spindexer);

  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spindexer.setKickMotor(1);
    System.out.println(x);
    x++;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    x = 0;
    System.out.println("done");
    spindexer.setKickMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return x>timer ;
  }
}
