/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/* ===========================================================================*/
/* Swerve Module Type 1                                                       */
/*                                                                            */
/* Drive Motor - Falcon Motor                                                 */
/* Drive Controller - TalonFX                                                 */
/* Drive Sensor - Falcon Motor                                                */
/* Steer Motor -                                                              */
/* Steer Controller - TalonSRX                                                */
/* Steer Sensor - MA2                                                         */
/* Steer Control Method wpiPID                                                */
/*                                                                            */
/* ===========================================================================*/

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_1;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_1.SwerveModule_Type_1_Constants.*;

public class SwerveModule_Type_1_SubSys extends SubsystemBase {
  /**
   * Creates a new FalconTalonFXDriveTalonSRXSteer_MA3_wpiPID_SwerveModule_SubSys
   */

  // Module String ID
  public String m_SwerveModuleStringID;

  // Module Specific 
  public SwerveModule_Type_1_ModuleConstants m_ModuleConstants;
  
  // Drive Motor
  public TalonFX m_DrvMtr;
    
  // Drive Motor PIDCotroller
  public PIDController m_DrvMtrPID = new PIDController(0, 0, 0);

  public double m_DrvMtrSetpoint = 0;
  public double m_DrvPIDMtrCmd = 0;

  public double m_DrvFFMtrCmd = 0;

  // Steer Motor
  public TalonSRX m_StrMtr;

  // Steer Motor Encoder
  public int m_SensorZeroPosCnts;

  // Steer Motor PIDCotroller
  public PIDController m_StrMtrPID = new PIDController(0, 0, 0);

  public double m_StrMtrSetpoint = 0;
  public double m_StrPIDMtrCmd = 0;

  // ShuffleBoard
  public SwerveModule_Type_1_Shuffleboard m_SwerveModuleShuffleboard;
  public boolean m_SwModule_ShuffleBoard_Enable;

  // Module State
  public SwerveModuleState m_SwerveModuleState;
  public SwerveModuleState m_SwerveModuleStateCmd;
  public SwerveModuleState m_OptimumSwerveModuleStateCmd;

  // Test Mode
  public boolean m_DrvMtr_PID_TestEnabled = false;
  public boolean m_StrMtr_PID_TestEnabled = false;

  /**
   * Constructs a FalconTalonFXDriveTalonSRXSteer_MA3_wpiPID_SwerveModule.
   *
   * @param swerveModuleStringID            String Swerve Module ID (FL,FR,RL,RR) 
   * @param drvMtr_ID                 int     ID for the drive motor
   * @param strMtr_ID                 int     ID for the turning motor
   */

  public SwerveModule_Type_1_SubSys(    
    String swerveModuleStringID,
    int drvMtr_ID,                       
    int strMtr_ID)
  {

    m_SwerveModuleStringID = swerveModuleStringID;

    switch(m_SwerveModuleStringID){
      case "FL":
          m_ModuleConstants = FL_Module.FL_Module_Settings;
          break;
      case "FR":
          m_ModuleConstants = FR_Module.FR_Module_Settings;
          break;
      case "RL":
          m_ModuleConstants = RL_Module.RL_Module_Settings;
          break;
      default:
          m_ModuleConstants = RR_Module.RR_Module_Settings;
    }

    // Setup Drive Motor
    m_DrvMtr = new TalonFX(drvMtr_ID);
    m_DrvMtr.configFactoryDefault();
    m_DrvMtr.setInverted(m_ModuleConstants.m_DrvMtr_Inverted);
    m_DrvMtr.setNeutralMode(DriveMotorConstants.NMode);

    // Setup Drive Encoder
    m_DrvMtr.configSelectedFeedbackSensor(DriveMotorConstants.FbkDevice);
    m_DrvMtr.configIntegratedSensorInitializationStrategy(DriveMotorConstants.SensorInitStrategy, 20);
    m_DrvMtr.setSensorPhase(m_ModuleConstants.m_DrvMtr_SensorPhaseInverted);
    m_DrvMtr.configFeedbackNotContinuous(DriveMotorConstants.FeedbackNotContinuous, 20);
    m_DrvMtr.configSelectedFeedbackCoefficient(DriveMotorConstants.FeedbackCoefficient);

    // Setup Drive PID Controller
    //m_DrvMtr.config_kP(DriveMotorConstants.ControllerID, DriveMotorConstants.P);
    //m_DrvMtr.config_kI(DriveMotorConstants.ControllerID, DriveMotorConstants.I);
    //m_DrvMtr.config_kD(DriveMotorConstants.ControllerID, DriveMotorConstants.D);
    //m_DrvMtr.configAllowableClosedloopError(DriveMotorConstants.ControllerID, DriveMotorConstants.AllowableClosedLoopError, 20);
    //m_DrvMtr.configClosedloopRamp(DriveMotorConstants.ClosedloopRamp);
    m_DrvMtrPID.setP(DriveMotorConstants.P);
    m_DrvMtrPID.setI(DriveMotorConstants.I);
    m_DrvMtrPID.setD(DriveMotorConstants.D);
    m_DrvMtrPID.setTolerance(
      DriveMotorConstants.AllowableClosedLoopError,
      DriveMotorConstants.AllowableClosedLoopDerError);


    // Setup Steer Motor
    m_StrMtr = new TalonSRX(strMtr_ID);
    m_StrMtr.configFactoryDefault();
    m_StrMtr.setInverted(m_ModuleConstants.m_StrMtr_Inverted);

    // Setup Steer Motor Encoder
    m_StrMtr.configSelectedFeedbackSensor(SteerMotorConstants.FdkDevice);
    m_StrMtr.configSelectedFeedbackCoefficient(SteerMotorConstants.FeedbackGain);
    m_StrMtr.configFeedbackNotContinuous(SteerMotorConstants.ConifgFeedbackNotContinuous, 20);
    m_StrMtr.setSensorPhase(m_ModuleConstants.m_StrMtr_SensorPhaseInverted);

    // Setup Turning Motor PID
    m_StrMtrPID.setP(SteerMotorConstants.P);
    m_StrMtrPID.setI(SteerMotorConstants.I);
    m_StrMtrPID.setD(SteerMotorConstants.D);
    m_StrMtrPID.enableContinuousInput(
      SteerMotorConstants.MinContinuousInput,
      SteerMotorConstants.MaxContinouseInput);
    m_StrMtrPID.setTolerance(
      SteerMotorConstants.AllowableClosedLoopError,
      SteerMotorConstants.AllowableClosedLoopDerError);
    m_StrMtrPID.setIntegratorRange(SteerMotorConstants.MinIntegral, SteerMotorConstants.MaxIntegral);

    m_SensorZeroPosCnts = m_ModuleConstants.m_StrMtr_SensorZeroPosCnts;

    // ShuffleBoard
    m_SwModule_ShuffleBoard_Enable = m_ModuleConstants.m_SwModule_ShuffleBoard_Enable;
    m_SwerveModuleShuffleboard = new SwerveModule_Type_1_Shuffleboard(
      m_SwerveModuleStringID);
    
    // Initialize States
    m_SwerveModuleState = getState();
    m_SwerveModuleStateCmd = new SwerveModuleState(0.0, new Rotation2d(0.0));
    m_OptimumSwerveModuleStateCmd = new SwerveModuleState(0.0, new Rotation2d(0.0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    if (m_SwModule_ShuffleBoard_Enable){
      if(m_SwerveModuleShuffleboard.getDrvMtrPID_TestEnable()){
        m_DrvMtr_PID_TestEnabled = true;
        m_DrvMtrPID.setP(m_SwerveModuleShuffleboard.getDrvMtrPID_P());
        m_DrvMtrPID.setI(m_SwerveModuleShuffleboard.getDrvMtrPID_I());
        m_DrvMtrPID.setD(m_SwerveModuleShuffleboard.getDrvMtrPID_D());
      }else{
        if(m_DrvMtr_PID_TestEnabled){
          m_DrvMtrPID.setP(DriveMotorConstants.P);
          m_DrvMtrPID.setI(DriveMotorConstants.I);
          m_DrvMtrPID.setD(DriveMotorConstants.D);
        }
      }
      if(m_SwerveModuleShuffleboard.getStrMtrPID_TestEnable()){
        m_StrMtr_PID_TestEnabled = true;
        m_StrMtrPID.setP(m_SwerveModuleShuffleboard.getStrMtrPID_P());
        m_StrMtrPID.setI(m_SwerveModuleShuffleboard.getStrMtrPID_I());
        m_StrMtrPID.setD(m_SwerveModuleShuffleboard.getStrMtrPID_D());
      }else{
        if(m_StrMtr_PID_TestEnabled){
          m_StrMtrPID.setP(DriveMotorConstants.P);
          m_StrMtrPID.setI(DriveMotorConstants.I);
          m_StrMtrPID.setD(DriveMotorConstants.D);
        }
      } 
      

      m_SwerveModuleShuffleboard.Display_SwerveModule(
        m_SwerveModuleStringID,
        m_DrvMtr,
        m_StrMtr,
        m_SensorZeroPosCnts,
        m_StrMtrPID,
        m_StrMtrSetpoint,
        m_StrPIDMtrCmd,
        m_SwerveModuleState,
        m_SwerveModuleStateCmd,
        m_OptimumSwerveModuleStateCmd);       
    }
  }

  /**
   * Returns the current state of the module.
   *
   * @return SwerveModuleState:  The current state of the module.
  */
  public SwerveModuleState getState() {

    double DriveMetersPerSec = DrvMtrCntsPer100ms2MeterPerSec(m_DrvMtr.getSelectedSensorVelocity());
    double SteerPosRads = StrMtrEncoderCnts2Rad(getEncoderCnts());
  
    // Update State
    m_SwerveModuleState = new SwerveModuleState(DriveMetersPerSec, new Rotation2d(SteerPosRads));
    return m_SwerveModuleState;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state SwerveModuleState:  Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    
    // Display Only
    m_SwerveModuleStateCmd = state;
    
    if (SwerveModuleConstants.OptimizedSwerveCmd){
      m_OptimumSwerveModuleStateCmd = CalculateOptimumStateCmd(state);
    } else {
      m_OptimumSwerveModuleStateCmd = state;  
    }
    
    //m_DrvMtr.set(ControlMode.Velocity, DrvMtrMeterPerSec2CntsPer100PerSec(m_OptimumSwerveModuleStateCmd.speedMetersPerSecond));  
    // Display Only
    m_DrvMtrSetpoint = m_OptimumSwerveModuleStateCmd.speedMetersPerSecond;

    m_DrvPIDMtrCmd = m_DrvMtrPID.calculate(
    getState().speedMetersPerSecond,
    m_OptimumSwerveModuleStateCmd.speedMetersPerSecond);

    m_DrvFFMtrCmd = m_OptimumSwerveModuleStateCmd.speedMetersPerSecond*DriveMotorConstants.FF_gain; 
    
    m_DrvMtr.set(ControlMode.PercentOutput, m_DrvPIDMtrCmd+m_DrvFFMtrCmd); 

    // Display Only
    m_StrMtrSetpoint = m_OptimumSwerveModuleStateCmd.angle.getRadians();

    m_StrPIDMtrCmd = m_StrMtrPID.calculate(
      getState().angle.getRadians(),
      m_OptimumSwerveModuleStateCmd.angle.getRadians());
      
    m_StrMtr.set(ControlMode.PercentOutput, m_StrPIDMtrCmd);  
  }

  /**
   * Zeros the Swerve Module.
   */
  public void resetSwerveModule() {
    m_DrvMtr.setSelectedSensorPosition(0);
    m_DrvMtrPID.reset();
    m_StrMtrPID.reset();
  }

  /**
   * Resets the Swerve PIDs
   */
  public void resetSwerveModulePID(){
    m_DrvMtrPID.reset();
    m_StrMtrPID.reset();
  }

  /** 
   * Drive Motor Helper function to convert m/s speed to Counts per 100ms
   * 
   * @param state SwerveModuleState Command
   * @return SwerveModuleState Optimized Swerve Module Command State 
   */
  private SwerveModuleState CalculateOptimumStateCmd(SwerveModuleState state){
    double StrAng0 = getState().angle.getRadians(); // Current Steer Angle
    double StrAngD = state.angle.getRadians(); // Desired Steer Angle
    //double StrAng0_pos = 0; // Current Steer Angle Positive Direction
    double StrAng0_opp = 0; // Current Steer Angle Negative Direction
    double AngDiff = 0;  // Difference in Angles
    SwerveModuleState StateCmd = new SwerveModuleState();

    StateCmd.angle = state.angle;
    StateCmd.speedMetersPerSecond = state.speedMetersPerSecond;

    /*
    if (StrAng0>=0) {
      if (StrAngD>=0) {
        AngDiff = StrAngD-StrAng0;
      } else {
        AngDiff = StrAngD-StrAng0;
      }
    } else {
      if (StrAngD>=0) {
        AngDiff = StrAngD-StrAng0;
      } else {
        AngDiff = StrAngD-StrAng0;
      }
    }
    */
    AngDiff = StrAngD-StrAng0;

    if(Math.abs(AngDiff)>(Math.PI/2+0.05)){
      if(StrAngD>=0){
        StateCmd.angle = new Rotation2d(StrAngD-Math.PI);
      }else{
        StateCmd.angle = new Rotation2d(StrAngD+Math.PI);
      }
      StateCmd.speedMetersPerSecond = -StateCmd.speedMetersPerSecond;
    }


    /*
    // Create Current Angle and Opposite
    if(StrAng0>=0){
      StrAng0_opp = StrAng0-Math.PI;
    }else{
      StrAng0_opp = StrAng0+Math.PI;
    }

    // Calculate Angle Difference from Current to Desired Options
    
    // Positive Desired Angle
    if(StrAngD>=0){
      AngDiff = Math.abs(StrAngD-StrAng0_pos);
      
      // Check if Current State is NOT Optimal
      if(AngDiff>((Math.PI/2)+0.05)){
        StateCmd.angle = new Rotation2d(StrAngD-Math.PI);
        StateCmd.speedMetersPerSecond = -state.speedMetersPerSecond;
      }

    // Negative Desired Angle
    }else{
      AngDiff = Math.abs(StrAngD-StrAng0_neg);

      // Check if Current State is NOT Optimal
      if(AngDiff>((Math.PI/2)+0.05)){
        StateCmd.angle = new Rotation2d(StrAngD+Math.PI);
        StateCmd.speedMetersPerSecond = -state.speedMetersPerSecond;
      }
    }
    */
    
    return StateCmd;
  }

  /** 
   * Drive Motor Helper function to convert m/s speed to Counts per 100ms
   * 
   * @param wheelMeterPerSec double Wheel Speed in Meters per second
   * @return int Drive Motor Counts Per 100ms 
   */
  private int DrvMtrMeterPerSec2CntsPer100PerSec (double wheelMeterPerSec) {
    double WheelShaftRevPerSec = wheelMeterPerSec/(Math.PI*DriveModuleConstants.WheelDiameter);
    double RevPerSec = WheelShaftRevPerSec*DriveModuleConstants.GearRatio;
    double CntsPerSec = RevPerSec*DriveMotorConstants.FbkEncoderCntsPerRev;
    int CntsPer100ms = (int) (CntsPerSec / 10);
    return CntsPer100ms;
  }

  /**
   * Drive Motor Helper function to convert Cnts/100ms to m/s Cmd
   * @param CntsPer100ms int Encoder Counts per 100ms
   * @return double Wheel Speed in m/s
   */
  private double DrvMtrCntsPer100ms2MeterPerSec (double CntsPer100ms) {
    double CntsPerSec = CntsPer100ms*10;
    double RevPerSec = CntsPerSec/DriveMotorConstants.FbkEncoderCntsPerRev;
    double WheelShaftRevPerSec = RevPerSec/DriveModuleConstants.GearRatio;
    double wheelMeterPerSec = WheelShaftRevPerSec * (Math.PI * DriveModuleConstants.WheelDiameter);
    return wheelMeterPerSec;
  }

  /**
   * Get Offset Encoder Counts (Uses m_SensorOffsetCnts)
   * @return Offset Encoder Counts
  */
  private double getEncoderCnts () {
    double OffsetEncoderCnts = m_StrMtr.getSelectedSensorPosition() - m_SensorZeroPosCnts;

    if (OffsetEncoderCnts < 0 ){
      OffsetEncoderCnts = OffsetEncoderCnts+ SteerMotorConstants.SensorCntsPerRev;
    }
    return OffsetEncoderCnts;
  }

  /**
   * Convert Offset Encoder Counts to Radians
   * @param encoderCnts int Encoder Counts
   * @return Steer Motor Radians (-pi - pi)
   */
  private double StrMtrEncoderCnts2Rad (double encoderCnts) {
    double strMtrRads = 0;
    if (encoderCnts > (SteerMotorConstants.SensorCntsPerRev/2)){
      strMtrRads = (encoderCnts - SteerMotorConstants.SensorCntsPerRev)*SteerMotorConstants.SensorCnts2Rads;
    } else {
      strMtrRads = encoderCnts*SteerMotorConstants.SensorCnts2Rads;
    }
    return strMtrRads;
  }

  /**
   * Get Max Drive Wheel Speed
   * @return maxDriveMtrWheelSpd double Maximum Drive Wheel Speed (m/s)
   */
  public double getMaxDriveWheelSpd(){
    return DriveModuleConstants.MaxDriveWheelSpd; 
  }

  public double getSteerRawPos(){
    return m_StrMtr.getSelectedSensorPosition();
  }
}
