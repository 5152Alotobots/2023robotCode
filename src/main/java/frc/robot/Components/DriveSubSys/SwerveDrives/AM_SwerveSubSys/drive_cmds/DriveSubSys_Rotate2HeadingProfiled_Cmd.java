// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Components.DriveSubSys.DriveSubSys_Old;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.SwerveDriveSubSys_Constants;
import edu.wpi.first.math.MathUtil;

public class DriveSubSys_Rotate2HeadingProfiled_Cmd extends CommandBase {
  
  private final double m_TargetAngleRads;
  private final DriveSubSys_Old m_DriveSubSys;
  private final ProfiledPIDController m_ProfiledPIDController;
  private final TrapezoidProfile.Constraints m_TConstraints;
  //private final TrapezoidProfile.State m_TPreviousState;
  //private final TrapezoidProfile m_TrapezoidProfile;

  /** Creates a new DriveSubSys_Rotate2Heading Command
  *  Turns to robot to the specified angle.
  *
  * @param targetAngleDegrees The angle to turn to degrees
  * @param driveSubSys        The drive subsystem to use
  */
  public DriveSubSys_Rotate2HeadingProfiled_Cmd(double targetAngleDegrees, DriveSubSys_Old driveSubSys) {
    
    m_TargetAngleRads = targetAngleDegrees*Math.PI/180;
    m_DriveSubSys = driveSubSys;
    m_TConstraints = new TrapezoidProfile.Constraints(.5, 3);
    //m_TPreviousState = new TrapezoidProfile.State(m_DriveSubSys.getHeading().getRadians(),0);

    m_ProfiledPIDController = new ProfiledPIDController(
      SwerveDriveSubSys_Constants.PGain_Rotation,
      SwerveDriveSubSys_Constants.IGain_Rotation,
      SwerveDriveSubSys_Constants.DGain_Rotation,
      m_TConstraints);
    
    m_ProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_ProfiledPIDController.setTolerance(1*Math.PI/180, 1*Math.PI/180);
    m_ProfiledPIDController.setIntegratorRange(-.3, 0.3);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ProfiledPIDController.reset(m_DriveSubSys.getHeading().getRadians());
    m_ProfiledPIDController.setGoal(m_TargetAngleRads);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double cmd = m_ProfiledPIDController.calculate(m_DriveSubSys.getHeading().getRadians(),m_TargetAngleRads);
    /*if (Math.abs(cmd)>m_DriveSubSys.getMaxDriveSubSysRotSpd()){
      cmd = MathUtil.clamp(
        cmd,
        -m_DriveSubSys.getMaxDriveSubSysRotSpd(),
        m_DriveSubSys.getMaxDriveSubSysRotSpd());

      m_ProfiledPIDController.reset(m_DriveSubSys.getHeading().getRadians(), 0);
    }
    */
    SmartDashboard.putNumber("ProfileSetPointPos", m_ProfiledPIDController.getGoal().position);
    //SmartDashboard.putNumber("ProfileSetPointVel", m_ProfiledPIDController.getSetpoint().velocity);
    SmartDashboard.putNumber("ProfiledCmd", cmd);
    m_DriveSubSys.Drive(0, 0, cmd, false, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_ProfiledPIDController.atSetpoint()){
      return true;
    } else {
      return false;
    }
  }
}
