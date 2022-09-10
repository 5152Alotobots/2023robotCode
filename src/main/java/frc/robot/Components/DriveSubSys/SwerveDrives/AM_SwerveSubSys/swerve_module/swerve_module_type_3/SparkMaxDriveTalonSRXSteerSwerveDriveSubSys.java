/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_3;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_3.*;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_3.SwerveModule_Type_3_Constants.DriveConstants;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_3.SwerveModule_Type_3_Constants.DriveConstants.DriveMotorConstants;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_3.SwerveModule_Type_3_Constants.DriveConstants.SteerMotorConstants;
import edu.wpi.first.wpilibj.SerialPort;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;

@SuppressWarnings("PMD.ExcessiveImports")
public class SparkMaxDriveTalonSRXSteerSwerveDriveSubSys extends SubsystemBase {
  /**
   * Creates a new SwerveDriveSubSys.
   */
    //Robot swerve modules
    private final SparkMaxDriveTalonSRXSteerSwerveDriveModule m_frontLeft = 
      new SparkMaxDriveTalonSRXSteerSwerveDriveModule(
        DriveMotorConstants.kFrontLeftPort,       // int Drive Mtr Port
        DriveMotorConstants.kFrontLeftInverted,   // boolean Drive Inverted
        SteerMotorConstants.kFrontLeftPort,       // int Turning Mtr Port
        SteerMotorConstants.kFrontLeftInverted,   // boolean Turning Mtr Port
        SteerMotorConstants.ksetSensorPhase);     // FeedBackDevice Turning Mtr Feedback Device                          
  
    private final SparkMaxDriveTalonSRXSteerSwerveDriveModule m_rearLeft =
      new SparkMaxDriveTalonSRXSteerSwerveDriveModule(
        DriveMotorConstants.kRearLeftPort,       // int Drive Mtr Port
        DriveMotorConstants.kRearLeftInverted,   // boolean Drive Inverted
        SteerMotorConstants.kRearLeftPort,       // int Turning Mtr Port
        SteerMotorConstants.kRearLeftInverted,   // boolean Turning Mtr Port
        SteerMotorConstants.ksetSensorPhase);     // FeedBackDevice Turning Mtr Feedback Device 

    private final SparkMaxDriveTalonSRXSteerSwerveDriveModule m_frontRight =
      new SparkMaxDriveTalonSRXSteerSwerveDriveModule(
        DriveMotorConstants.kFrontRightPort,       // int Drive Mtr Port
        DriveMotorConstants.kFrontRightInverted,   // boolean Drive Inverted
        SteerMotorConstants.kFrontRightPort,       // int Turning Mtr Port
        SteerMotorConstants.kFrontRightInverted,   // boolean Turning Mtr Port
        SteerMotorConstants.ksetSensorPhase);     // FeedBackDevice Turning Mtr Feedback Device 

    private final SparkMaxDriveTalonSRXSteerSwerveDriveModule m_rearRight =
      new SparkMaxDriveTalonSRXSteerSwerveDriveModule(
        DriveMotorConstants.kRearRightPort,       // int Drive Mtr Port
        DriveMotorConstants.kRearRightInverted,   // boolean Drive Inverted
        SteerMotorConstants.kRearRightPort,       // int Turning Mtr Port
        SteerMotorConstants.kRearRightInverted,   // boolean Turning Mtr Port
        SteerMotorConstants.ksetSensorPhase);     // FeedBackDevice Turning Mtr Feedback Device 
    
    // Odometry class for tracking robot pose
    public SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getAngle());
  
    // NavX
    private AHRS m_navXGyro;

    /**
     * Creates a new DriveSubsystem.
     */
    public SparkMaxDriveTalonSRXSteerSwerveDriveSubSys() {

      try {
        /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
        /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
        /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      m_navXGyro = new AHRS(I2C.Port.kMXP); 
      } catch (RuntimeException ex ) {
        DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
      }
      
      //m_navXGyro.reset();
    }
  
    /**
     * Returns the angle of the robot as a Rotation2d.
     *
     * @return The angle of the robot.
     */
    public Rotation2d getAngle() {
      // Negating the angle because WPILib gyros are CW positive.
      //double test = m_navXGyro.getAngle();

      //return Rotation2d.fromDegrees(m_navXGyro.getAngle() * (DriveConstants.kGyroReversed ? 1.0 : -1.0));
      return Rotation2d.fromDegrees(0);
    }
  
    @Override
    public void periodic() {
      // Update the odometry in the periodic block
      /*m_odometry.update(
          new Rotation2d(getHeading()),
          m_frontLeft.getState(),
          m_rearLeft.getState(),
          m_frontRight.getState(),
          m_rearRight.getState());
      */
      
      SmartDashboard.putNumber("RawGyro", m_navXGyro.getAngle());
      //SmartDashboard.putNumber("GyroOffset", m_navXGyro.getAngleAdjustment());
      SmartDashboard.putNumber("GetAngle", getAngle().getDegrees());
      /*
      double StrRadPosCmd = 0;
      SmartDashboard.putNumber("StrRadPosCmd",StrRadPosCmd);
      double StrEncPosCmd = m_frontRight.StrMtrRads2EncoderCnts(StrRadPosCmd);
      SmartDashboard.putNumber("StrEncPosCmd",StrEncPosCmd);

      m_frontRight.m_turningMotor.set(ControlMode.Position, StrEncPosCmd); 
      m_frontLeft.m_turningMotor.set(ControlMode.Position, StrEncPosCmd); 
      m_rearRight.m_turningMotor.set(ControlMode.Position, StrEncPosCmd); 
      m_rearLeft.m_turningMotor.set(ControlMode.Position, StrEncPosCmd); 
      */
      SmartDashboard.putNumber("StrEncPos", m_frontRight.m_turningMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("StrRadPos", m_frontRight.getState().angle.getRadians());
      /*
      double DrvMetersPerSecCmd = 0.5;
      SmartDashboard.putNumber("DrvMetersPerSecCmd",DrvMetersPerSecCmd);
      double DrvRPM = m_frontRight.DrvMtrMeterPerSec2RPM(DrvMetersPerSecCmd);
      SmartDashboard.putNumber("DrvRPMCmd", DrvRPM);

      m_frontRight.m_drivePIDController.setReference(DrvRPM, ControlType.kVelocity);
      SwerveModuleState tempModuleState = new SwerveModuleState();
      tempModuleState.speedMetersPerSecond = 0.5;
      Rotation2d tempAngleCmd = new Rotation2d(Math.PI/2);
      tempModuleState.angle = tempAngleCmd;
      m_frontRight.setDesiredState(tempModuleState);
      */
      SmartDashboard.putNumber("DrvRPM", m_frontRight.m_driveEncoder.getVelocity());
      SmartDashboard.putNumber("DrvMetersPerSec", m_frontRight.getState().speedMetersPerSecond);
    }
  
    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    //public Pose2d getPose() {
      //return m_odometry.getPoseMeters();
    //}
  
    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    //public void resetOdometry(Pose2d pose) {
    //  m_odometry.resetPosition(pose, getAngle());
    //}
  
    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
          fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
              //xSpeed, ySpeed, rot, getAngle())
              xSpeed, ySpeed, rot, Rotation2d.fromDegrees(0))
              : new ChassisSpeeds(xSpeed, ySpeed, rot)
      );
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,DriveConstants.kMaxSpeedMetersPerSecond);
      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_rearLeft.setDesiredState(swerveModuleStates[2]);
      m_rearRight.setDesiredState(swerveModuleStates[3]);

      SmartDashboard.putNumber("FR_SpdCmd",swerveModuleStates[0].speedMetersPerSecond);
      SmartDashboard.putNumber("FR_StrCmd",swerveModuleStates[0].angle.getRadians());
    }
  
    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
      SwerveModule_Type_3_Constants.DriveConstants.kMaxSpeedMetersPerSecond);
      m_frontLeft.setDesiredState(desiredStates[0]);
      m_frontRight.setDesiredState(desiredStates[1]);
      m_rearLeft.setDesiredState(desiredStates[2]);
      m_rearRight.setDesiredState(desiredStates[3]);
    }
  
    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
      m_frontLeft.resetDriveMotorEncoder();
      m_rearLeft.resetDriveMotorEncoder();
      m_frontRight.resetDriveMotorEncoder();
      m_rearRight.resetDriveMotorEncoder();
    }
  
    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
      //m_navXGyro.reset();
    }
  
    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeading() {
      //return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
      //return m_navXGyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
      return 0;
    }
  
    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
    //  return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
      //return m_navXGyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
      return 0;
    }
  }