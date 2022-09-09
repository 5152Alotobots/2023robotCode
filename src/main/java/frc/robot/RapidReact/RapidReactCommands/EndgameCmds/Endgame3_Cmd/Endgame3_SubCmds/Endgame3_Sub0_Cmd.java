// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.RapidReactCommands.EndgameCmds.Endgame3_Cmd.Endgame3_SubCmds;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RobotSettings;
import frc.robot.RapidReact.Climber.ClimberSubSys;
import frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftSubSys;
import frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftCmds.ClimberLift_FullLoadSlot1_Pos_Cmd;
import frc.robot.RapidReact.Climber.ClimberRotator.ClimberRotatorSubSys;
import frc.robot.RapidReact.Intake.IntakeArm.IntakeArmSubSys;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Endgame3_Sub0_Cmd extends SequentialCommandGroup {
  /** Creates a new BasicAuto_Cmd. */
  public Endgame3_Sub0_Cmd(
    ClimberSubSys climberSubSys,
    ClimberLiftSubSys climberLiftSubSys,
    ClimberRotatorSubSys climberRotatorSubSys,
    IntakeArmSubSys intakeArmSubSys
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      // Current SubStep Commands
      // Climb Up Rung 2
      new ClimberLift_FullLoadSlot1_Pos_Cmd(
        climberLiftSubSys,
        TalonFXControlMode.Position,
        RobotSettings.Climbing.kLift2Rung2StopPos)
        .withTimeout(5),
      
      new InstantCommand(climberSubSys::incrementClimbState, climberSubSys),

      // Next SubStep Command
      new Endgame3_Sub1_Cmd(climberSubSys, climberLiftSubSys, climberRotatorSubSys)
    );
  }
}
