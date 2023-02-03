// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.ChargedUp.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotSettings;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriveTrains.SwerveDrive.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.htm


public class Auto_YOUBETTERWORKORELSE_Cmd extends SequentialCommandGroup {
  private final SubSys_DriveTrain m_DriveTrain; 
  /** Creates a new Auto_Challenge1_Cmd. */
  public Auto_YOUBETTERWORKORELSE_Cmd( 
    SubSys_DriveTrain driveSubSys
     ){
      m_DriveTrain = driveSubSys;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
  
  
      )
      ;
  }
}

  