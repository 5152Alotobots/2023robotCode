// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Components.DriveSubSys.DriveSubSys_Old;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.SwerveDriveSubSys_Constants;
import edu.wpi.first.math.MathUtil;

public class DriveSubSys_Rotate2Heading_Cmd extends CommandBase {

  private final double m_TargetAngleRads;
  private final DriveSubSys_Old m_DriveSubSys;
  private final PIDController m_PIDController;

  /** Creates a new DriveSubSys_Rotate2Heading Command
  *  Turns to robot to the specified angle.
  *
  * @param targetAngleDegrees The angle to turn to degrees
  * @param driveSubSys        The drive subsystem to use
  */
  public DriveSubSys_Rotate2Heading_Cmd(double targetAngleDegrees, DriveSubSys_Old driveSubSys) {
    
    m_TargetAngleRads = targetAngleDegrees*Math.PI/180;
    m_DriveSubSys = driveSubSys;
 
    m_PIDController = new PIDController(
      SwerveDriveSubSys_Constants.PGain_Rotation,
      SwerveDriveSubSys_Constants.IGain_Rotation,
      SwerveDriveSubSys_Constants.DGain_Rotation);
    
    m_PIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_PIDController.setTolerance(1*Math.PI/180, 1*Math.PI/180);
    m_PIDController.setIntegratorRange(-.3, 0.3);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PIDController.setSetpoint(m_TargetAngleRads);
    m_PIDController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double cmd = m_PIDController.calculate(m_DriveSubSys.getHeading().getRadians());
    if (Math.abs(cmd)>m_DriveSubSys.getMaxDriveSubSysRotSpd()){
      cmd = MathUtil.clamp(
        cmd,
        -m_DriveSubSys.getMaxDriveSubSysRotSpd(),
        m_DriveSubSys.getMaxDriveSubSysRotSpd());

      m_PIDController.reset();
    }
    m_DriveSubSys.Drive(0, 0, cmd, false, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_PIDController.atSetpoint()){
      return true;
    } else {
      return false;
    }
  }
}
