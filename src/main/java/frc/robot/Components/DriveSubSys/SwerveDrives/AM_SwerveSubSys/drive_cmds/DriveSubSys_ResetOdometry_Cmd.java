// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Components.DriveSubSys.DriveSubSys_Old;
import edu.wpi.first.math.geometry.Pose2d;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveSubSys_ResetOdometry_Cmd extends InstantCommand {

  private final DriveSubSys_Old m_DriveSubSys;
  private final Pose2d m_Pose;
  
  public DriveSubSys_ResetOdometry_Cmd(
    DriveSubSys_Old driveSubSys,
    Pose2d pose){
    
      m_DriveSubSys = driveSubSys;
      m_Pose = pose;
      addRequirements(driveSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveSubSys.resetOdometry(m_Pose);    
  }
}
