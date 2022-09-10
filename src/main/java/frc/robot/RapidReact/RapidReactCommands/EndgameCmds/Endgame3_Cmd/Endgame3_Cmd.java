// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.RapidReactCommands.EndgameCmds.Endgame3_Cmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RapidReact.Climber.ClimberSubSys;
import frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftSubSys;
import frc.robot.RapidReact.Climber.ClimberRotator.ClimberRotatorSubSys;
import frc.robot.RapidReact.Intake.IntakeArm.IntakeArmSubSys;
import frc.robot.RapidReact.RapidReactCommands.EndgameCmds.Endgame3_Cmd.Endgame3_SubCmds.Endgame3_Sub0_Cmd;
import frc.robot.RapidReact.RapidReactCommands.EndgameCmds.Endgame3_Cmd.Endgame3_SubCmds.Endgame3_Sub1_Cmd;
import frc.robot.RapidReact.RapidReactCommands.EndgameCmds.Endgame3_Cmd.Endgame3_SubCmds.Endgame3_Sub2_Cmd;
import frc.robot.RapidReact.RapidReactCommands.EndgameCmds.Endgame3_Cmd.Endgame3_SubCmds.Endgame3_Sub3_Cmd;
import frc.robot.RapidReact.RapidReactCommands.EndgameCmds.Endgame3_Cmd.Endgame3_SubCmds.Endgame3_Sub4_Cmd;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Endgame3_Cmd extends SequentialCommandGroup {
  /** Creates a new BasicAuto_Cmd. */
  public Endgame3_Cmd(
    ClimberSubSys climberSubSys,
    ClimberLiftSubSys climberLiftSubSys,
    ClimberRotatorSubSys climberRotatorSubSys,
    IntakeArmSubSys intakeArmSubSys) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    switch(climberSubSys.getClimbState()){
      case 0:
        addCommands(
          new Endgame3_Sub0_Cmd(climberSubSys, climberLiftSubSys, climberRotatorSubSys, intakeArmSubSys)
        );
        break;
        
      case 1:
        addCommands(
          new Endgame3_Sub1_Cmd(climberSubSys, climberLiftSubSys, climberRotatorSubSys)
        );
        break;

      case 2:
        addCommands(
          new Endgame3_Sub2_Cmd(climberSubSys, climberLiftSubSys, climberRotatorSubSys)
        );
        break;

      case 3:
        addCommands(
          new Endgame3_Sub3_Cmd(climberSubSys, climberLiftSubSys, climberRotatorSubSys)
        );
        break;

      case 4:
        addCommands(
          new Endgame3_Sub4_Cmd(climberSubSys, climberLiftSubSys, climberRotatorSubSys)
        );
        break;

      default:
        addCommands();
    }
  }
}
