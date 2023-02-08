/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Library.DriveTrains.AutonSubsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotSettings;
import frc.robot.Library.DriveTrains.SwerveDrive.SubSys_SwerveDrive;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Library.DriveTrains.*;
import frc.robot.Library.DriveTrains.SwerveDrive.*;

public class SubSys_Hurry extends CommandBase {
  /** Creates a new DriveSubSys_DriveTrajectory1_Cmd. */

  private final SubSys_DriveTrain m_DriveTrain;
  private PathPlannerTrajectory m_Test;
  private final Timer m_Timer = new Timer();
  
  public SubSys_Hurry(SubSys_DriveTrain driveSubSys) {
    
    m_DriveTrain = driveSubSys;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain);
    
    //String trajectoryJSON = "output/BarrelRace.wpilib.json";
    //m_TrajectoryPathweaver = new Trajectory();
    //try {
    //  Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //  m_TrajectoryPathweaver = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    //} catch (IOException ex) {
    //  DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    //}
    //Translation2d translation = new Translation2d(2.40, Rotation2d.fromDegrees(-108)); 
    //Transform2d transform = new Transform2d(translation, Rotation2d.fromDegrees(0));
    
    //m_TrajectoryPathweaver = m_TrajectoryPathweaver.transformBy(transform);

    m_Test = PathPlanner.loadPath("Hurry", 1, 1);

    /*
    m_HolonomicController = new HolonomicDriveController(
      new PIDController(
        SubSys_SwerveDrive_Constants.PGain_Translation,
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
        */

  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d initialPose = m_Test.getInitialHolonomicPose();
    
    SmartDashboard.putNumber("PP_X0", initialPose.getX());
    SmartDashboard.putNumber("PP_Y0", initialPose.getY());
    SmartDashboard.putNumber("PP_Rot0", initialPose.getRotation().getDegrees());

    m_DriveTrain.resetOdometry(initialPose);
    m_Timer.reset();
    m_Timer.start();

    SmartDashboard.putNumber("Timer_X0", m_Timer.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = m_Timer.get();
    PathPlannerState goal = (PathPlannerState) m_Test.sample(time); 

    SmartDashboard.putNumber("Timer", time);
    SmartDashboard.putNumber("PP_X", goal.poseMeters.getX());
    SmartDashboard.putNumber("PP_Y", goal.poseMeters.getY());
    //SmartDashboard.putNumber("PP_Rotation", goal.poseMeters.getRotation().getDegrees());
    SmartDashboard.putNumber("PP_Rotation", goal.holonomicRotation.getDegrees());
/*
    ChassisSpeeds adjustedSpeeds = m_HolonomicController.calculate(
      m_DriveTrain.getPose(),
      goal,
      goal.holonomicRotation);

    SmartDashboard.putNumber("XVel_Cmd", adjustedSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("YVel_Cmd", adjustedSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Rot_Cmd", adjustedSpeeds.omegaRadiansPerSecond);
*/
    //m_DriveTrain.Drive(
    //  adjustedSpeeds.vxMetersPerSecond, 
    //  adjustedSpeeds.vyMetersPerSecond,
    //  adjustedSpeeds.omegaRadiansPerSecond, true, false, false);
  
    /*
    m_DriveSubSys.Drive(
      adjustedSpeeds.vxMetersPerSecond, 
      adjustedSpeeds.vyMetersPerSecond,
      0, //adjustedSpeeds.omegaRadiansPerSecond, 
      true, false, false);
    */
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Timer.reset();
    m_DriveTrain.Drive(0, 0, 0, false, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Timer.get()>= (m_Test.getTotalTimeSeconds()+10)) {
      return true;
    }else{
      return false;
    } 
  }

}

