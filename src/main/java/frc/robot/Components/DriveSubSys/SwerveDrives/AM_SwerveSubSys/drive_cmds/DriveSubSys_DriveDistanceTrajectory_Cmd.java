// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.drive_cmds;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.Components.DriveSubSys.DriveSubSys_Constants;
import frc.robot.Components.DriveSubSys.DriveSubSys_Old;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.SwerveDriveSubSys_Constants;

public class DriveSubSys_DriveDistanceTrajectory_Cmd extends CommandBase {
  /** Creates a new DriveSubSys_DriveTrajectory1_Cmd. */

  private final DriveSubSys_Old m_DriveSubSys;
  private final Trajectory m_Trajectory1;
  private final Timer m_Timer = new Timer();
  private final HolonomicDriveController m_HolonomicController;
  private final double m_Distance;
  
  public DriveSubSys_DriveDistanceTrajectory_Cmd(DriveSubSys_Old driveSubSys, double distance) {
    
    m_DriveSubSys = driveSubSys;
    m_Distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubSys);
    
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(.5,     //m_DriveSubSys.getMaxDriveSubSysSpd(),
                             .25)     //m_DriveSubSys.getMaxDriveSubSysAccel())
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(m_DriveSubSys.getSwerveDriveKinematics());
    /*
    // Trajectory 1  All units in meters.
    m_Trajectory1 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(.3333, .3333),
            new Translation2d(.6666, -.3333)
        ),
        // End 1 meters straight ahead of where we started, facing forward
        new Pose2d(1, 0, new Rotation2d(0)),
        // Pass config
        config       
    );
    */

    // Trajectory 1  All units in meters.
    m_Trajectory1 = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
          new Translation2d(m_Distance/2, 0)
        ),
        // End 1 meters straight ahead of where we started, facing forward
        new Pose2d(m_Distance, 0, new Rotation2d(0)),
        // Pass config
        config       
    );
    
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
    m_Timer.reset();
    m_Timer.start();
    m_DriveSubSys.resetDrivePIDs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State goal = m_Trajectory1.sample(m_Timer.get()); 

    ChassisSpeeds adjustedSpeeds = m_HolonomicController.calculate(
      m_DriveSubSys.getPose(),
      goal,
      goal.poseMeters.getRotation());

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
    if (m_Timer.get()>= (m_Trajectory1.getTotalTimeSeconds()+1)) {
      return true;
    }else{
      return false;
    } 
  }
}
