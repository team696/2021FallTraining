/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import org.team696.robot.subsystems.Drivetrain;
import org.team696.robot.subsystems.Shooter;

public class Drive extends CommandBase {
  private final Drivetrain drivetrain;
  private final DoubleSupplier speedSupplier;
  private final DoubleSupplier turnSupplier;

  /**
   * Creates a new Drive.
   */
  double currentServoPosition;

  public Drive(DoubleSupplier speed, DoubleSupplier turn, Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.speedSupplier = speed;
    this.turnSupplier = turn;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // currentServoPosition = shooter.servoAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(speedSupplier.getAsDouble(), turnSupplier.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
