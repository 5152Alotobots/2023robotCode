// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Components.DriveSubSys.DriveSubSys_Old;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.SwerveDriveSubSys_Constants;
import frc.robot.Constants.RobotSettings;

public class DriveSubSys_DriveVariableTrajectory2_Cmd extends CommandBase {
  /** Creates a new DriveSubSys_DriveTrajectory1_Cmd. */

  private DriveSubSys_Old m_DriveSubSys;
  private Trajectory m_Trajectory;
  private Timer m_Timer = new Timer();
  private HolonomicDriveController m_HolonomicController;
  private Pose2d m_InitialPose;
  private List<Translation2d> m_WayPointPoses;
  private Pose2d m_FinalPose;
  private boolean m_InitCurrPose;
  private PIDController m_XDirPID;
  private PIDController m_YDirPID;
  private ProfiledPIDController m_RotPID;
  
  public DriveSubSys_DriveVariableTrajectory2_Cmd(
    DriveSubSys_Old driveSubSys, 
    Pose2d initialPose,
    List<Translation2d> wayPointPoses,
    Pose2d finalPose,
    boolean initCurrPose) {
    
    m_DriveSubSys = driveSubSys;
    m_InitialPose = initialPose;
    m_WayPointPoses = wayPointPoses;
    m_FinalPose = finalPose;
    m_InitCurrPose = initCurrPose;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubSys);
    
    // Create config for trajectory
    TrajectoryConfig config =
      new TrajectoryConfig(
        RobotSettings.Drive.kTrajMaxVel,
        RobotSettings.Drive.kTrajMaxAccel)
        .setKinematics(m_DriveSubSys.getSwerveDriveKinematics());

    // Trajectory
    if (m_InitCurrPose){
      m_Trajectory = TrajectoryGenerator.generateTrajectory(
        m_DriveSubSys.getPose(),
        m_WayPointPoses,
        m_FinalPose,
        config);
    } else {
      m_Trajectory = TrajectoryGenerator.generateTrajectory(
        m_InitialPose,
        m_WayPointPoses,
        m_FinalPose,
        config);
    }
    /*
    m_Trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(new Translation2d(0,0), new Rotation2d(0)),
        List.of(
          new Translation2d(1,0),
          new Translation2d(2,0)),
        new Pose2d(new Translation2d(3,0), new Rotation2d(0)),
        config);
     */
    m_XDirPID = new PIDController(
      SwerveDriveSubSys_Constants.PGain_Translation,
      SwerveDriveSubSys_Constants.IGain_Translation,
      SwerveDriveSubSys_Constants.DGain_Translation);

    m_YDirPID = new PIDController(
      SwerveDriveSubSys_Constants.PGain_Translation,
      SwerveDriveSubSys_Constants.IGain_Translation,
      SwerveDriveSubSys_Constants.DGain_Translation);

    m_RotPID = new ProfiledPIDController(
      SwerveDriveSubSys_Constants.PGain_Rotation,
      SwerveDriveSubSys_Constants.IGain_Rotation,
      SwerveDriveSubSys_Constants.DGain_Rotation,
      new TrapezoidProfile.Constraints(3, 3));

    m_HolonomicController = new HolonomicDriveController(
      m_XDirPID,
      m_YDirPID,
      m_RotPID);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    m_DriveSubSys.resetDrivePIDs();
    m_XDirPID.reset();
    m_YDirPID.reset();
    //m_RotPID.reset(
    //  m_DriveSubSys.getPose().getRotation().getRadians(), 0.0);
    Trajectory.State goal = m_Trajectory.sample(m_Timer.get()); 

    ChassisSpeeds adjustedSpeeds = m_HolonomicController.calculate(
      m_DriveSubSys.getPose(),
      goal,
      m_InitialPose.getRotation());
      //goal.poseMeters.getRotation());
    m_HolonomicController.setEnabled(true);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State goal = m_Trajectory.sample(m_Timer.get()); 

    ChassisSpeeds adjustedSpeeds = m_HolonomicController.calculate(
      m_DriveSubSys.getPose(),
      goal,
      m_FinalPose.getRotation());
      //goal.poseMeters.getRotation());

    m_DriveSubSys.Drive(
      adjustedSpeeds.vxMetersPerSecond, 
      adjustedSpeeds.vyMetersPerSecond,
      adjustedSpeeds.omegaRadiansPerSecond, 
      true, false, false);
    
    SmartDashboard.putNumber("XGoal", goal.poseMeters.getTranslation().getX());
    SmartDashboard.putNumber("YGoal", goal.poseMeters.getTranslation().getY());
    SmartDashboard.putNumber("HeadingGoal", m_RotPID.getGoal().position);
    //SmartDashboard.putNumber("HeadingGoal", m_FinalPose.getRotation().getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_HolonomicController.setEnabled(false);
    m_Timer.reset();
    m_DriveSubSys.Drive(0, 0, 0, false, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Timer.get()>= (m_Trajectory.getTotalTimeSeconds()+1)) {
      return true;
    }else{
      return false;
    } 
  }
}
