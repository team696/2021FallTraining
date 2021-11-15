// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team696.robot.commands;

import org.team696.robot.Constants.IntakeConstants;
import org.team696.robot.subsystems.Intake;
import org.team696.robot.subsystems.Limelight;
import org.team696.robot.subsystems.Shooter;
import org.team696.robot.subsystems.ShooterHood;
import org.team696.robot.subsystems.TurretSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullShoot extends ParallelCommandGroup {
  // TurretSubsystem turretSubsystem;
  // Shooter shooter;
  // Intake intake;
  // ShooterHood shooterHood;
  // Limelight limelight;

  // /** Creates a new FullShoot. */
  // public FullShoot(TurretSubsystem turretSubsystem, Shooter shooter, Intake intake, ShooterHood shooterHood, Limelight limelight) {
  //   this.turretSubsystem = turretSubsystem;
  //   this.shooter = shooter;
  //   this.intake = intake;
  //   this.shooterHood = shooterHood;
  //   this.limelight = limelight;

  //   addRequirements(turretSubsystem, shooter, intake, shooterHood, limelight);
  //   // Add your commands in the addCommands() call, e.g.
  //   // addCommands(new FooCommand(), new BarCommand());
  //   addCommands(new ShooterHoodCommand(shooterHood, 51), new TurretLockOn(turretSubsystem, limelight), new ShooterCommand(shooter, shooter.shootRPM, true, intake, IntakeConstants.intakeMidPosition) );
  }
// }
