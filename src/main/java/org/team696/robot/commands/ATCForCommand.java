/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import org.team696.robot.Constants;
import org.team696.robot.subsystems.Spindexer;

public class ATCForCommand extends CommandBase {
  /**
   * Creates a new SpinToIndex.
   */
  private static final Logger logger = LogManager.getLogger(ATCForCommand.class);
  Spindexer spindexer;

  int oldSpindexerPocket = 0;

  public ATCForCommand(Spindexer spindexer) {
    this.spindexer = spindexer;
    addRequirements(spindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oldSpindexerPocket = spindexer.getTargetPocket();
    int targetPocket;
    if (spindexer.getTargetPocket() == Constants.SpindexerConstants.nPockets) {
      targetPocket = 1;
    } else {
      targetPocket = spindexer.getTargetPocket() + 1;
    }
    logger.info(String.format("Moving spindexer from %d to %d", oldSpindexerPocket, targetPocket));
    spindexer.setTargetPocket(targetPocket);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      logger.info("Spindexer command interrupted.");
    } else {
      logger.info("Spindexer on target.");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return spindexer.isOnTarget();
  }
}
