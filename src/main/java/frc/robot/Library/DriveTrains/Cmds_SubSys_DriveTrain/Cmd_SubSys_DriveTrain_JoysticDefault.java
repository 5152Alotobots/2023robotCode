/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriverStation.JoystickUtilities;

public class Cmd_SubSys_DriveTrain_JoysticDefault extends CommandBase {
  /**
   * Creates a new FalconTalonFXDriveTalonSR.
   */
  private final SubSys_DriveTrain driveSubSys;
  private final DoubleSupplier fwdCmd;
  private final DoubleSupplier strCmd;
  private final DoubleSupplier rotCmd;
  private final boolean fieldOriented;
  private final BooleanSupplier rotateLeftPt;
  private final BooleanSupplier rotateRightPt;

  // Max Speed Command is 4.7mps

  public Cmd_SubSys_DriveTrain_JoysticDefault(
    SubSys_DriveTrain driveSubSys,
    DoubleSupplier fwdCmd,
    DoubleSupplier strCmd,
    DoubleSupplier rotCmd,
    boolean fieldOriented,
    BooleanSupplier rotateLeftPt,
    BooleanSupplier rotateRightPt){

    this.driveSubSys = driveSubSys;
    this.fwdCmd = fwdCmd;
    this.strCmd = strCmd;
    this.rotCmd = rotCmd;
    this.fieldOriented = fieldOriented;
    this.rotateLeftPt = rotateLeftPt;
    this.rotateRightPt = rotateRightPt;
    addRequirements(driveSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubSys.Drive(
      JoystickUtilities.joyDeadBndSqrdScaled(fwdCmd.getAsDouble(), 0.05,driveSubSys.getMaxDriveSubSysSpd()),
      JoystickUtilities.joyDeadBndSqrdScaled(strCmd.getAsDouble(),0.05,driveSubSys.getMaxDriveSubSysSpd()),
      //JoystickUtilities.joyDeadBndScaled(fwdCmd.getAsDouble(),0.1,m_DriveSubSys.getMaxDriveSubSysSpd()),
      //JoystickUtilities.joyDeadBndScaled(m_StrCmd.getAsDouble(),0.1,m_DriveSubSys.getMaxDriveSubSysSpd()),
      JoystickUtilities.joyDeadBndScaled(rotCmd.getAsDouble(),0.1,driveSubSys.getMaxDriveSubSysRotSpd()),
      fieldOriented,
      rotateLeftPt.getAsBoolean(),
      rotateRightPt.getAsBoolean());

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
