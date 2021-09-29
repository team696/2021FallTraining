/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team696.robot.subsystems.Shooter;
import org.team696.robot.subsystems.Spindexer;

public class FireCommand extends CommandBase {
  /**
   * Creates a new FireCommand.
   */
  private Shooter shooter;
  private Spindexer spindexer;
  private double power;

  public FireCommand(Shooter shooter, Spindexer spindexer, double kickMotorPower) {

    this.shooter = shooter;
    this.spindexer = spindexer;
    this.power = kickMotorPower;
    addRequirements(shooter);
    addRequirements(spindexer);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.isUpToSpeed()){
      // shooter.setAcceleratorPower(true);
      spindexer.setKickMotor(power);
      // spindexer.spindexerLoadingAntiJam(0.5, 20);
    

    }
    else{
      spindexer.setKickMotor(0);

    }
    
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
