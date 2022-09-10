// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Components.DriveSubSys.DriveSubSys_Old;

public class DriveSubSys_Drive4Time_Cmd extends CommandBase {
  /** Creates a new DriveSubSys_Drive4Time_Cmd. */
  private final DriveSubSys_Old m_DriveSubSys;
  private final double m_DriveFwdSpd;
  private final double m_DriveStrSpd;
  private final double m_DriveRotSpd;
  private final double m_DriveTime;
  private final boolean m_FieldOriented;
  private final Timer m_timer = new Timer();
  

  public DriveSubSys_Drive4Time_Cmd(
    DriveSubSys_Old driveSubSys,
    double driveFwdSpd,
    double driveStrSpd,
    double driveRotSpd,
    double driveTime,
    boolean fieldOriented){

      m_DriveSubSys = driveSubSys;
      m_DriveFwdSpd = driveFwdSpd;
      m_DriveStrSpd = driveStrSpd;
      m_DriveRotSpd = driveRotSpd;
      m_DriveTime = driveTime;
      m_FieldOriented = fieldOriented;

      addRequirements(driveSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveSubSys.Drive(m_DriveFwdSpd, m_DriveStrSpd, m_DriveRotSpd, m_FieldOriented, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.reset();
    m_DriveSubSys.Drive(0, 0, 0, m_FieldOriented, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.get()>= m_DriveTime) {
      return true;
    }else{
      return false;
    } 
  }
}
