// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.RapidReactCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Components.DriveSubSys.DriveSubSys_Old;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds.DriveSubSys_SeekRotate2LimeLightTarget_Cmd;
import frc.robot.Components.Vision.LimeLightSubSys;
import frc.robot.RapidReact.Intake.IntakeArm.IntakeArmSubSys;
import frc.robot.RapidReact.Intake.IntakeInNOut.IntakeInNOutSubSys;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot_Cmd extends ParallelCommandGroup {
  /** Creates a new ShooterAutoTarget_Cmd. */
  public AutoShoot_Cmd(
    DriveSubSys_Old driveSubSys,
    IntakeArmSubSys intakeArmSubSys,
    IntakeInNOutSubSys intakeInNOutSubSys,
    LimeLightSubSys limeLightSubSys,
    DoubleSupplier fwdCmd,
    DoubleSupplier strCmd    
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveSubSys_SeekRotate2LimeLightTarget_Cmd(
        driveSubSys, 
        fwdCmd, 
        strCmd, 
        true, limeLightSubSys),

      new ShooterAutoTarget_Cmd(
        driveSubSys,
        intakeArmSubSys,
        intakeInNOutSubSys, 
        limeLightSubSys)
    );
  }
}
