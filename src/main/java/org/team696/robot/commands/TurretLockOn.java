/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team696.robot.subsystems.Limelight;
import org.team696.robot.subsystems.TurretSubsystem;

public class TurretLockOn extends CommandBase {
  /**
   * Creates a new TurretLockOn.
   */
  
   TurretSubsystem turretSubsystem;
   Limelight limelight;
   public TurretLockOn(TurretSubsystem turretSubsystem, Limelight limelight) {
    this.turretSubsystem = turretSubsystem;
    this.limelight = limelight;
    addRequirements(turretSubsystem);
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setLights(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // turretSubsystem.openLoop(0.05);
    turretSubsystem.moveToTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return turretSubsystem.onTarget();
    return false;
  }
}
