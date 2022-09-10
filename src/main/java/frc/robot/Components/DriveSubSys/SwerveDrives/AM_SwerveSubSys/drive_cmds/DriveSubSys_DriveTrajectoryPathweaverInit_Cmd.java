// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Components.DriveSubSys.DriveSubSys_Constants;
import frc.robot.Components.DriveSubSys.DriveSubSys_Old;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.SwerveDriveSubSys_Constants;

public class DriveSubSys_DriveTrajectoryPathweaverInit_Cmd extends CommandBase {
  /** Creates a new DriveSubSys_DriveTrajectory1_Cmd. */

  private final DriveSubSys_Old m_DriveSubSys;
  private Trajectory m_TrajectoryPathweaver;
  private final Timer m_Timer = new Timer();
  private final HolonomicDriveController m_HolonomicController;
  
  public DriveSubSys_DriveTrajectoryPathweaverInit_Cmd(DriveSubSys_Old driveSubSys) {
    
    m_DriveSubSys = driveSubSys;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubSys);
    
    String trajectoryJSON = "output/Slalom.wpilib.json";
    m_TrajectoryPathweaver = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      m_TrajectoryPathweaver = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    //Translation2d translation = new Translation2d(2.40, Rotation2d.fromDegrees(-108)); 
    //Transform2d transform = new Transform2d(translation, Rotation2d.fromDegrees(0));
    
    //m_TrajectoryPathweaver = m_TrajectoryPathweaver.transformBy(transform);
    
    m_HolonomicController = new HolonomicDriveController(
      new PIDController(
        SwerveDriveSubSys_Constants.PGain_Translation,
        SwerveDriveSubSys_Constants.IGain_Translation,
        SwerveDriveSubSys_Constants.DGain_Translation),
      new PIDController(
        SwerveDriveSubSys_Constants.PGain_Translation,
        SwerveDriveSubSys_Constants.IGain_Translation,
        SwerveDriveSubSys_Constants.DGain_Translation),
      new ProfiledPIDController(
        SwerveDriveSubSys_Constants.PGain_Rotation,
        SwerveDriveSubSys_Constants.IGain_Rotation,
        SwerveDriveSubSys_Constants.DGain_Rotation,
        new TrapezoidProfile.Constraints(1, 3)));   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_DriveSubSys.zeroGyro();
    m_DriveSubSys.resetOdometry(m_TrajectoryPathweaver.getInitialPose());
    m_Timer.reset();
    m_Timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State goal = m_TrajectoryPathweaver.sample(m_Timer.get()); 

    ChassisSpeeds adjustedSpeeds = m_HolonomicController.calculate(
      m_DriveSubSys.getPose(),
      goal,
      //goal.poseMeters.getRotation());
      Rotation2d.fromDegrees(0));

    m_DriveSubSys.Drive(
      adjustedSpeeds.vxMetersPerSecond, 
      adjustedSpeeds.vyMetersPerSecond,
      adjustedSpeeds.omegaRadiansPerSecond, false, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Timer.reset();
    m_DriveSubSys.Drive(0, 0, 0, false, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Timer.get()>= (m_TrajectoryPathweaver.getTotalTimeSeconds()+1)) {
      return true;
    }else{
      return false;
    } 
  }
}
