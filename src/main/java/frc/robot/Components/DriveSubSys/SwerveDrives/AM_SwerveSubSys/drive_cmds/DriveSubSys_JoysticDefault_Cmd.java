/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Components.DriveSubSys.DriveSubSys_Old;
import frc.robot.Components.DriverStation.JoystickUtilities;

public class DriveSubSys_JoysticDefault_Cmd extends CommandBase {
  /**
   * Creates a new FalconTalonFXDriveTalonSR.
   */
  private final DriveSubSys_Old m_DriveSubSys;
  private final DoubleSupplier m_FwdCmd;
  private final DoubleSupplier m_StrCmd;
  private final DoubleSupplier m_RotCmd;
  private final boolean m_FieldOriented;
  private final BooleanSupplier m_RotateLeftPt;
  private final BooleanSupplier m_RotateRightPt;

  // Max Speed Command is 4.7mps

  public DriveSubSys_JoysticDefault_Cmd(
    DriveSubSys_Old driveSubSys,
    DoubleSupplier fwdCmd,
    DoubleSupplier strCmd,
    DoubleSupplier rotCmd,
    boolean fieldOriented,
    BooleanSupplier rotateLeftPt,
    BooleanSupplier rotateRightPt){

    m_DriveSubSys = driveSubSys;
    m_FwdCmd = fwdCmd;
    m_StrCmd = strCmd;
    m_RotCmd = rotCmd;
    m_FieldOriented = fieldOriented;
    m_RotateLeftPt = rotateLeftPt;
    m_RotateRightPt = rotateRightPt;
    addRequirements(driveSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveSubSys.Drive(
      JoystickUtilities.joyDeadBndSqrdScaled(m_FwdCmd.getAsDouble(), 0.05,m_DriveSubSys.getMaxDriveSubSysSpd()),
      JoystickUtilities.joyDeadBndSqrdScaled(m_StrCmd.getAsDouble(),0.05,m_DriveSubSys.getMaxDriveSubSysSpd()),
      //JoystickUtilities.joyDeadBndScaled(m_FwdCmd.getAsDouble(),0.1,m_DriveSubSys.getMaxDriveSubSysSpd()),
      //JoystickUtilities.joyDeadBndScaled(m_StrCmd.getAsDouble(),0.1,m_DriveSubSys.getMaxDriveSubSysSpd()),
      JoystickUtilities.joyDeadBndScaled(m_RotCmd.getAsDouble(),0.1,m_DriveSubSys.getMaxDriveSubSysRotSpd()),
      m_FieldOriented,
      m_RotateLeftPt.getAsBoolean(),
      m_RotateRightPt.getAsBoolean());

      //SmartDashboard.putBoolean("RotateLeft_JoyCmd", m_RotateLeftPt.getAsBoolean());
      //SmartDashboard.putBoolean("RotateRight_JoyCmd", m_RotateRightPt.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
