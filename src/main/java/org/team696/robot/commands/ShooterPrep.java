/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import org.team696.robot.subsystems.Limelight;
import org.team696.robot.subsystems.Shooter;
import org.team696.robot.subsystems.TurretSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterPrep extends CommandBase {

  //maybe add omni to this command or make it part of the spindexer drum one
  TurretSubsystem turretSubsystem;
  Shooter shooter;
  public ShooterPrep(Shooter shooter, TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.shooter = shooter;
    addRequirements(shooter);
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Limelight.setLights(3);
    turretSubsystem.moveToTarget();
    shooter.setShooterVelocity(shooter.shooterRPMToTalonVelocity(shooter.getShootRPM()));
    shooter.setAcceleratorPower(true);
    //maybe add shooter hood
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
