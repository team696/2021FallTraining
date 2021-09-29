/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import org.team696.robot.subsystems.Limelight;
import org.team696.robot.subsystems.ShooterHood;
import org.team696.robot.subsystems.Spindexer;
import org.team696.robot.subsystems.TurretSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FullShoot extends SequentialCommandGroup {
  /**
   * Creates a new FullShoot.
   */
  public FullShoot(ShooterHood shooterHood, Spindexer spindexer, TurretSubsystem turretSubsystem, Limelight limelight) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new ShooterHoodCommand(shooterHood, 51),
      new SpinToPocket(spindexer, 1),
      new TurretLockOn(turretSubsystem, limelight),
      new OmniKickUpTimer(spindexer, true, 50),
      new SpinToPocket(spindexer, 2),
      new OmniKickUpTimer(spindexer, true, 50),
      new SpinToPocket(spindexer, 3),
      new OmniKickUpTimer(spindexer, true, 50),
      new SpinToPocket(spindexer, 4),
      new OmniKickUpTimer(spindexer, true, 50),
      new SpinToPocket(spindexer, 5),
      new OmniKickUpTimer(spindexer, true, 50),
      new ShooterHoodCommand(shooterHood, 10)

    );
  }
}
