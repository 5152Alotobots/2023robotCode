// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.RapidReactCommands.EndgameCmds.Endgame3_Cmd.Endgame3_SubCmds;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotSettings;
import frc.robot.RapidReact.Climber.ClimberSubSys;
import frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftSubSys;
import frc.robot.RapidReact.Climber.ClimberRotator.ClimberRotatorSubSys;
import frc.robot.RapidReact.Climber.ClimberRotator.ClimberRotatorCmds.ClimberRotator_Pos_Cmd;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Endgame3_Sub1_Cmd extends SequentialCommandGroup {
  /** Creates a new BasicAuto_Cmd. */
  public Endgame3_Sub1_Cmd(
    ClimberSubSys climberSubSys,
    ClimberLiftSubSys climberLiftSubSys,
    ClimberRotatorSubSys climberRotatorSubSys
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      // Current SubStep Commands
      
      // Rotate Up to Rung 3
      new ClimberRotator_Pos_Cmd(
        climberRotatorSubSys,
        TalonFXControlMode.MotionMagic,
        RobotSettings.Climbing.kRung2toRung3Angle,
        true, 0).withTimeout(2),
    
      //new ClimberLift_Pos_Cmd(climberLiftSubSys, TalonFXControlMode.MotionMagic, 2000),
      new InstantCommand(climberSubSys::incrementClimbState, climberSubSys),

      // Next SubStep Command
      new Endgame3_Sub2_Cmd(climberSubSys, climberLiftSubSys, climberRotatorSubSys)
    );
  }
}
