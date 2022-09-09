/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_3;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_3.SwerveModule_Type_3_Constants.*;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_3.SwerveModule_Type_3_Constants.DriveConstants.DriveModuleConstants;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_3.SwerveModule_Type_3_Constants.DriveConstants.DriveMotorConstants;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_3.SwerveModule_Type_3_Constants.DriveConstants.SteerMotorConstants;

public class SparkMaxDriveTalonSRXSteerSwerveDriveModule {
  public CANSparkMax m_driveMotor;
  public TalonSRX m_turningMotor;
  public CANPIDController m_drivePIDController;
  public CANEncoder m_driveEncoder;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SparkMaxDriveTalonSRXSteerSwerveDriveModule(int driveMotorChannel, boolean driveMotorInverted,
    int turningMotorChannel, boolean turningMotorInverted, boolean turningMotorSetSensorPhase){

    // Setup Drive Motor    
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setInverted(driveMotorInverted);
    
    // Setup Drive Encoder
    m_driveEncoder = m_driveMotor.getEncoder();
    
    // Setup Drive PID Controller
    m_drivePIDController = m_driveMotor.getPIDController();
    m_drivePIDController.setP(DriveMotorConstants.kP, DriveMotorConstants.kPID_ID);
    m_drivePIDController.setI(DriveMotorConstants.kI, DriveMotorConstants.kPID_ID);
    m_drivePIDController.setD(DriveMotorConstants.kD, DriveMotorConstants.kPID_ID);

    // Setup Turning Motor
    m_turningMotor = new TalonSRX(turningMotorChannel);
    m_turningMotor.setInverted(turningMotorInverted);
    m_turningMotor.configSelectedFeedbackSensor(SteerMotorConstants.kFeedbackDevice);
    m_turningMotor.configSelectedFeedbackCoefficient(SteerMotorConstants.kFeedbackGain);
    m_turningMotor.configFeedbackNotContinuous(SteerMotorConstants.kconifgFeedbackNotContinuous, SteerMotorConstants.kconfigFeedbackNotContinuousTimeOut);
    m_turningMotor.setSensorPhase(turningMotorSetSensorPhase);
    m_turningMotor.config_kP(SteerMotorConstants.kControllerID, SteerMotorConstants.kP);
    m_turningMotor.config_kI(SteerMotorConstants.kControllerID, SteerMotorConstants.kI);
    m_turningMotor.config_kD(SteerMotorConstants.kControllerID, SteerMotorConstants.kD);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {

    double DriveMetersPerSec = DrvMtrRPM2MeterPerSec(m_driveEncoder.getVelocity());
    

    double divisor = Math.floor(m_turningMotor.getSelectedSensorPosition()/1024);
    double TurningMotorSensorPositionRemainder = m_turningMotor.getSelectedSensorPosition()-(divisor*1024);
    double SteerPosRads = StrMtrEncoderCnts2Rad(TurningMotorSensorPositionRemainder);
  
    return new SwerveModuleState(DriveMetersPerSec, new Rotation2d(SteerPosRads));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
 
    m_drivePIDController.setReference(DrvMtrMeterPerSec2RPM(state.speedMetersPerSecond), ControlType.kVelocity);  
    m_turningMotor.set(ControlMode.Position, StrMtrRads2EncoderCnts(state.angle.getRadians())); 
  }
  /**)
   * Drive Motor Helper function to convert m/s speed Cmd to RPM cmd
   */
  public double DrvMtrMeterPerSec2RPM (double WheelMPerSecCmd) {
    double WheelShaftRevPerSecCmd = WheelMPerSecCmd/(Math.PI*DriveModuleConstants.kWheelDiameter);
    double RevPerSecCmd = WheelShaftRevPerSecCmd*DriveModuleConstants.kGearRatio;
    double RPMCmd = RevPerSecCmd*60; 
    return RPMCmd;
  }

    /**
   * Drive Motor Helper function to convert RPM Cmd to m/s Cmd
   */
  public double DrvMtrRPM2MeterPerSec (double RPMCmd) {
    double RevPerSecCmd = RPMCmd/60;
    double WheelShaftRevPerSecCmd = RevPerSecCmd/DriveModuleConstants.kGearRatio;
    double WheelMPerSecCmd = WheelShaftRevPerSecCmd * (Math.PI * DriveModuleConstants.kWheelDiameter);
    return WheelMPerSecCmd;
  }


  /**
   * Steer Motor Helper function to convert Radian Cmd to Encoder Cnts
   */
  public int StrMtrRads2EncoderCnts (double radCmd) {
    double EncoderCntsCmd = radCmd*SteerMotorConstants.kSensorRads2Cnts;
    return (int) Math.floor(EncoderCntsCmd);
  }

    /**
   * Steer Motor Helper function to convert Encoder Counts Cmd to Radian Cmd
   */
  public double StrMtrEncoderCnts2Rad (double encoderCmd) {
    return encoderCmd*SteerMotorConstants.kSensorCnts2Rads;
  }

  /**
   * Zeros the Swerve Drive Motor encoder.
   */
  public void resetDriveMotorEncoder() {
    m_driveEncoder.setPosition(0.0);
  }
}