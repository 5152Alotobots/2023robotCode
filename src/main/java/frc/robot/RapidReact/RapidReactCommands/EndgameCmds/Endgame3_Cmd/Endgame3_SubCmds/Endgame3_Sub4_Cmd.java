// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.RapidReactCommands.EndgameCmds.Endgame3_Cmd.Endgame3_SubCmds;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RobotSettings;
import frc.robot.RapidReact.Climber.ClimberSubSys;
import frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftSubSys;
import frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftCmds.ClimberLift_FullLoadAngledSlot2_Pos_Cmd;
import frc.robot.RapidReact.Climber.ClimberRotator.ClimberRotatorSubSys;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Endgame3_Sub4_Cmd extends SequentialCommandGroup {
  public Endgame3_Sub4_Cmd(
    ClimberSubSys climberSubSys,
    ClimberLiftSubSys climberLiftSubSys,
    ClimberRotatorSubSys climberRotatorSubSys
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      // Current SubStep Commands
      // Climb Up Rung 4
      new ClimberLift_FullLoadAngledSlot2_Pos_Cmd(
        climberLiftSubSys,
        TalonFXControlMode.MotionMagic,
        RobotSettings.Climbing.kTraverse2Rung4StopPos)
        .withTimeout(10),

      new InstantCommand(climberSubSys::resetClimbState, climberSubSys),
      
      new WaitCommand(5)
      
      // Next SubStep Command
    );
  }
}
