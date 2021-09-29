/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import org.team696.robot.subsystems.Intake;
import org.team696.robot.subsystems.Spindexer;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoIndexKickUp extends SequentialCommandGroup {
  /**
   * Creates a new AutoIndexKickUp.
   */
  public AutoIndexKickUp(Spindexer spindexer, Intake intake, double drumPower, double omniPower) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      // new ATCForCommand(spindexer),
        // new OmniKickUp(spindexer, omniPower),
        new ContinuousShoot(spindexer, drumPower, true)
      
    );
  }
}
