// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Components.DriveSubSys.DriveSubSys_Old;
import frc.robot.Components.Vision.LimeLightSubSys;

public class DriveSubSys_Range2LimeLightTarget_Cmd extends CommandBase {
  /** Creates a new DriveSubSys_Rotate2LimeLightTarget_Cmd. */
  private DriveSubSys_Old m_DriveSubSys;
  private LimeLightSubSys m_LimeLightSubSys;
  private double KpRange = .04;
  private double FFRangeCmd = 0.25;  // m/s 
  //private double targetdistance; //distance (in meters)
  //private double targetdistance;
  private double m_targetdistance;

  public DriveSubSys_Range2LimeLightTarget_Cmd(
    DriveSubSys_Old driveSubSys,
    LimeLightSubSys limeLightSubSys, double targetdistance) {

    m_DriveSubSys = driveSubSys;
    m_LimeLightSubSys = limeLightSubSys;
    m_targetdistance = targetdistance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubSys);
    addRequirements(limeLightSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
    // Creates a new flat moving average filter
    // Average will be taken over the last 5 samples
    LinearFilter filter = LinearFilter.movingAverage(5);
    double ty = filter.calculate(m_LimeLightSubSys.m_Ty);

    double distance = 2.64/(Math.tan(Math.abs(ty))); 
    double range_error = distance-m_targetdistance;
    double rangeCmd = 0.0;

    if (range_error > 0.1 && ty>0)
    {
      rangeCmd = KpRange*range_error - FFRangeCmd;
    }else if(range_error > 0.1 && ty<0){

      rangeCmd = -(KpRange*range_error - FFRangeCmd);
    }
    else if (range_error < -0.1 && ty>0)
    {
      rangeCmd = KpRange*range_error + FFRangeCmd;
    }
    else if (range_error < -0.1 && ty<0)
    {
      rangeCmd = -(KpRange*range_error + FFRangeCmd);
    }

    m_DriveSubSys.Drive(
      rangeCmd, 0, 0, false, false, false);
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
    if (Math.abs(m_LimeLightSubSys.m_Ty)<1){
      return true;
    } else {
    return false;
    }
  }
}
