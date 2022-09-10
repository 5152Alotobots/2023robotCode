// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Components.DriveSubSys.DriveSubSys_Old;
import frc.robot.Components.DriverStation.JoystickUtilities;
import frc.robot.Components.Vision.LimeLightSubSys;

public class DriveSubSys_Rotate2LimeLightTarget_Cmd extends CommandBase {
  /** Creates a new DriveSubSys_Rotate2LimeLightTarget_Cmd. */
  private DriveSubSys_Old m_DriveSubSys;
  private LimeLightSubSys m_LimeLightSubSys;
  private final DoubleSupplier m_FwdCmd;
  private final DoubleSupplier m_StrCmd;
  private final boolean m_FieldOriented;
  private double FFRotCmd = 1.6;  //1.5  Rads/s 
  

  private ProfiledPIDController m_RotateController;
  private double PGain_Rotation = 1.0; //0.18
  private double IGain_Rotation = 0.2;
  private double DGain_Rotation = 0.00;

  public DriveSubSys_Rotate2LimeLightTarget_Cmd(
    DriveSubSys_Old driveSubSys,
    DoubleSupplier fwdCmd,
    DoubleSupplier strCmd,
    boolean fieldOriented,
    LimeLightSubSys limeLightSubSys) {

    m_DriveSubSys = driveSubSys;
    m_FwdCmd = fwdCmd;
    m_StrCmd = strCmd;
    m_FieldOriented = fieldOriented;
    m_LimeLightSubSys = limeLightSubSys;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubSys);
    addRequirements(limeLightSubSys);

    m_RotateController = new ProfiledPIDController(
      PGain_Rotation,
      IGain_Rotation,
      DGain_Rotation,
      new TrapezoidProfile.Constraints(3, .75));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_RotateController.setTolerance(.1);
 }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
       // LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
    // Creates a new flat moving average filter
    // Average will be taken over the last 5 samples
    //LinearFilter filter = LinearFilter.movingAverage(10);
    //double tx = filter.calculate(m_LimeLightSubSys.m_Tx);

    double rotatecommand = m_RotateController.calculate(
      m_LimeLightSubSys.m_Tx, 0);

      
    /*
    if (m_LimeLightSubSys.m_Tx > 1.3)
    {
     // rotCmd  = Math.min(MaxSpd,(KpRot*heading_error - FFRotCmd));
     rotatecommand = rotatecommand-FFRotCmd;
    }
    else if(m_LimeLightSubSys.m_Tx > .2){
      rotatecommand = rotatecommand-.4;
    }
    else if(m_LimeLightSubSys.m_Tx < -.2){
      rotatecommand = rotatecommand+.4;
    }
    else if (m_LimeLightSubSys.m_Tx < -1.3)
    {
    //  rotCmd = Math.min(MaxSpd,KpRot*heading_error + FFRotCmd);
    rotatecommand = rotatecommand+FFRotCmd;
    }
    */
    SmartDashboard.putNumber("RotateCmd", rotatecommand);

    m_DriveSubSys.Drive(
      JoystickUtilities.joyDeadBndSqrdScaled(m_FwdCmd.getAsDouble(), 0.05,m_DriveSubSys.getMaxDriveSubSysSpd()),
      JoystickUtilities.joyDeadBndSqrdScaled(m_StrCmd.getAsDouble(),0.05,m_DriveSubSys.getMaxDriveSubSysSpd()),  
      rotatecommand, 
      true, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubSys.Drive(
      0, 0, 0, false, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if (Math.abs(m_LimeLightSubSys.m_Tx)<0.15){
    //  return true;
    //} else {
    return false;
    //}
  }
}
