/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// package org.team696.robot.commands;


// import org.team696.robot.subsystems.ClimberSubsystem;
// import org.team696.robot.RobotContainer;
// import org.team696.robot.Constants.OperatorConstants;
// import org.team696.robot.subsystems.ClimberServos;

// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class ClimberLift extends CommandBase {
//   private ClimberSubsystem climberSubsystem;
//   private ClimberServos climberServos;
//   /**
//    * Creates a new ClimberPullUp.
//    */
//   public ClimberLift(ClimberSubsystem climberSubsystem, ClimberServos climberServos) {
//     this.climberSubsystem = climberSubsystem;
//     this.climberServos = climberServos;
//     addRequirements(climberSubsystem);
//     addRequirements(climberServos);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//       // servoSubsystem.closeServo();
//       // if (RobotContainer.operatorPanel.getRawButton(OperatorConstants.climberUpButton)){
        
//         climberSubsystem.setClimberPower();
//         // System.out.println(climberSubsystem.climberTargetPower);
 
//   //  }
//   //  else{
//   //    climberSubsystem.setClimberPower(0);
//   //   // System.out.println(climberSubsystem.climberTargetPower);

//   //  }

  
//       }
      
    

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     // servoSubsystem.CloseServo();

//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
