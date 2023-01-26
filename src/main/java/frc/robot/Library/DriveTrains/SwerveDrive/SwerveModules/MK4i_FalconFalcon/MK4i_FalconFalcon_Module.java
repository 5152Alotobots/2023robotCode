// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* ===========================================================================*/
/* Swerve and Steer Falcon Drive and PG71 Steer                               */
/*                                                                            */
/* Drive Motor - Falcon Motor                                                 */
/* Drive Controller - TalonFX                                                 */
/* Drive Sensor - Falcon Motor                                                */
/* Drive Control Method - FalconFX                                            */
/* Steer Motor - Falcon Motor                                                 */
/* Steer Controller - TalonFX                                                 */
/* Steer Sensor - CANCoder                                                    */
/* Steer Control Method - TalonFX                                             */
/*                                                                            */
/* ===========================================================================*/

package frc.robot.Library.DriveTrains.SwerveDrive.SwerveModules.MK4i_FalconFalcon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Library.DriveTrains.SwerveDrive.SwerveModules.CTREModuleState;
import frc.robot.Library.DriveTrains.SwerveDrive.SwerveModules.SwerveModuleConstants;
import frc.robot.Library.DriveTrains.SwerveDrive.SwerveModules.MK4i_FalconFalcon.MK4i_FalconFalcon_Module_Constants.*;
import frc.robot.Library.MotorControllers.TalonFX.TalonFX_Conversions;

/** Add your docs here. */
public class MK4i_FalconFalcon_Module {

    public String moduleName;
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANCoder steerAngleEncoder;
    private double strZeroAngle;  
    private double lastAngle;
    private MK4i_FalconFalcon_Module_Constants mk4i_Module_Constants;

    //public int moduleNumber;

    SimpleMotorFeedforward driveMotorFF = new SimpleMotorFeedforward(
        DriveMotor.driveKS,
        DriveMotor.driveKV,
        DriveMotor.driveKA);

    /** MK4i_FalconFalcon_Module Constructor
     * 
     * @param moduleName String Name of Module FL,FR,BL,BR
     * @param moduleConstants SwerveModuleConstants Contains CAN ID's and Basic Motor Settings
     */
    public MK4i_FalconFalcon_Module(
        String moduleName,
        SwerveModuleConstants moduleConstants)
        {
            this.moduleName = moduleName;
            this.strZeroAngle = moduleConstants.zeroAngle;
            this.mk4i_Module_Constants = new MK4i_FalconFalcon_Module_Constants();

            /* Drive Motor Config */
            this.driveMotor = new TalonFX(moduleConstants.driveMotorID);
            configDriveMotor(moduleConstants);         

            /* Angle Encoder Config */
            this.steerAngleEncoder = new CANCoder(moduleConstants.cancoderID);
            configSteerAngleEncoder(moduleConstants);

            /* Steer Motor Config */
            this.steerMotor = new TalonFX(moduleConstants.steerMotorID);
            configSteerMotor(moduleConstants);
        
            lastAngle = getState().angle.getDegrees();
        }

    /***********************************************************************************/
    /* ***** Swerve Module Methods *****                                               */
    /***********************************************************************************/

    /** setDesiredState
     *   Sets the desired state of the Swerve Module
     * 
     * @param desiredState SwerveModuleState Desired state
     * @param isOpenLoop boolean Open Loop control for Drive
     */
    public void setDesiredState(
        SwerveModuleState desiredState,
        boolean isOpenLoop)
        {
            desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

            /* Drive Motor Command */
            // Open Loop 
            if(isOpenLoop){
                double percentOutput = desiredState.speedMetersPerSecond / Constants.RobotSettings.DriveTrain.DriveTrainMaxSpd;
                driveMotor.set(ControlMode.PercentOutput, percentOutput);
            }
            // Closed Loop
            else {
                double velocity = TalonFX_Conversions.MPSToFalcon(
                    desiredState.speedMetersPerSecond,
                    MK4i_FalconFalcon_Module_Constants.DriveMotor.driveWheelCircumference,
                    MK4i_FalconFalcon_Module_Constants.DriveMotor.driveGearRatio);

                driveMotor.set(
                    ControlMode.Velocity,
                    velocity,
                    DemandType.ArbitraryFeedForward,
                    driveMotorFF.calculate(desiredState.speedMetersPerSecond));
            }

            /* Steer Motor Command
             * Always Closed Loop
             * If Drive Speed Command is less than 1%, do not rotate to prevent jittering
            */

            // Set default angle to lastAngle
            double angle = lastAngle;
            if (Math.abs(desiredState.speedMetersPerSecond) > (Constants.RobotSettings.DriveTrain.DriveTrainMaxSpd * 0.01)){
                angle = desiredState.angle.getDegrees();
            }
            
            // Set Steet Motor Command to angle
            steerMotor.set(
                ControlMode.Position,
                TalonFX_Conversions.degreesToTalonFX(angle));
                
            // Update last angle         
            lastAngle = angle;
        }

    /** getState
     *    Gets the current state of the module
     * @return SwerveModuleState State of the Module
     */
    public SwerveModuleState getState(){
        /** Drive Wheel Velocity */
        double velocity = TalonFX_Conversions.falconToMPS(
            driveMotor.getSelectedSensorVelocity(),
            MK4i_FalconFalcon_Module_Constants.DriveMotor.driveWheelCircumference,
            MK4i_FalconFalcon_Module_Constants.DriveMotor.driveGearRatio);

        /** Steer Motor Angle */
        Rotation2d angle = getSteerAngle();
        return new SwerveModuleState(velocity, angle);
    }

    /** Serve Module Position
    * Returns the current position of the module
    * @return ServeModulePosition:  The current position of the module
    */
      public SwerveModulePosition getPosition(){
        SwerveModulePosition swrModulePosition;
        
        swrModulePosition = new SwerveModulePosition(
            getDriveMotorDistance(),
            getSteerAngle());

      return swrModulePosition;
    }

    /***********************************************************************************/
    /* ***** Drive Motor Methods *****                                                 */  
    /***********************************************************************************/

    /** configDriveMotor
     *    Configurations for the Drive Motor are defined in SubSys_SwerveDrive_Constants.java
     *    This is basic information like Motor and Sensor CAN ID's, Inverted and Steer Zero Angle
     * @param moduleConstants Module Constants
     */
    private void configDriveMotor(SwerveModuleConstants moduleConstants){        
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(this.mk4i_Module_Constants.getDriveMotorConfig());
        driveMotor.setInverted(moduleConstants.driveMotorInvert);
        driveMotor.setNeutralMode(MK4i_FalconFalcon_Module_Constants.DriveMotor.neutralMode);
    }

    /** getDriveMotorPosition
     *   Get Drive Motor Sensor Position
     * @return double Integrated Sensor Counts
     */
    public double getDriveMotorPosition(){
        return driveMotor.getSelectedSensorPosition();
    }

    /** getDriveWheelRevs
     *   Get Drive Motor 
     * @return double Drive Wheel Revs
     */
    public double getDriveWheelRevs(){
        
        double wheelRevs = TalonFX_Conversions.talonFXToRevs_GearRatio(
            getDriveMotorPosition(),
            MK4i_FalconFalcon_Module_Constants.DriveMotor.invDriveGearRatio);
        return wheelRevs;
    }

    /** getDriveWheelDistance
     *   Get Drive Wheel Distance
     * @return double Drive Wheel Distance (m)
     */
    public double getDriveMotorDistance(){
        double wheelRevs = TalonFX_Conversions.talonFXToRevs_GearRatio(
            getDriveMotorPosition(),
            MK4i_FalconFalcon_Module_Constants.DriveMotor.invDriveGearRatio);
        return wheelRevs*MK4i_FalconFalcon_Module_Constants.DriveMotor.driveWheelCircumference;
    }

    public void setDriveMotorPos(double counts){
        driveMotor.setSelectedSensorPosition(counts, 0, 0);
    }

    /***********************************************************************************/
    /* ***** Steer Angle Encoder Methods *****                                         */
    /***********************************************************************************/
    /**  configSteerAngleEncoder
     *      Configure Steer Angle Encoder
     * @param moduleConstants
     */
    private void configSteerAngleEncoder(SwerveModuleConstants moduleConstants){
        steerAngleEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        steerAngleEncoder.configSensorDirection(moduleConstants.SteerCANcoderInvert);
        double strAngleInit = steerAngleEncoder.getAbsolutePosition()-this.strZeroAngle;
        SmartDashboard.putNumber("strAngleInit", strAngleInit);
        //steerAngleEncoder.setPosition((steerAngleEncoder.getAbsolutePosition()-this.strZeroAngle));
        steerAngleEncoder.setPosition(strAngleInit);
    }
    
    /** getSteerSensorAbsolutePos
     *   Returns the Absolute Position Measurement
     * @return double Absolute Steer Sensor Positions Measurement (Degrees) 
     */
    public double getSteerSensorAbsolutePos(){
        return steerAngleEncoder.getAbsolutePosition();
    }

    /** getSteerSensorPos
     *   Returns the Position Measurement
     * @return double Steer Sensor Position Measurement 
     */
    public double getSteerSensorPos(){
        return steerAngleEncoder.getPosition();
    }

    /***********************************************************************************/
    /* ***** Steer Motor Methods *****                                                 */
    /***********************************************************************************/
    /** configSteerMotor
     * 
     * @param moduleConstants
     */
    private void configSteerMotor(SwerveModuleConstants moduleConstants){
        steerMotor.configFactoryDefault();
        steerMotor.configAllSettings(this.mk4i_Module_Constants.getSteerMotorConfig());
        steerMotor.setInverted(moduleConstants.steerMotorInvert);
        steerMotor.setNeutralMode(MK4i_FalconFalcon_Module_Constants.SteerMotor.neutralMode);

        steerMotor.configSelectedFeedbackSensor(MK4i_FalconFalcon_Module_Constants.SteerMotor.selectedFeedbackSensor);
        steerMotor.configSelectedFeedbackCoefficient(MK4i_FalconFalcon_Module_Constants.SteerMotor.selectedFeedbackCoefficient);
        steerMotor.configFeedbackNotContinuous(MK4i_FalconFalcon_Module_Constants.SteerMotor.feedbackNotContinuous, 0);
        steerMotor.setSensorPhase(MK4i_FalconFalcon_Module_Constants.SteerMotor.sensorPhase);

        // Config Internal Encoder
        steerMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        steerMotor.configIntegratedSensorOffset(0.0);

        // Config Remote Encoder
        steerMotor.configRemoteFeedbackFilter(steerAngleEncoder, 0);
        steerMotor.setSensorPhase(MK4i_FalconFalcon_Module_Constants.SteerCANcoder.sensorPhase);
        steerMotor.configSelectedFeedbackCoefficient(MK4i_FalconFalcon_Module_Constants.SteerCANcoder.selectedFeedbackCoefficient);

        steerMotor.configSelectedFeedbackSensor(
            FeedbackDevice.RemoteSensor0, 
            0, 
            0);

        steerMotor.config_kP(0, 0.6, 0);
    }

    /** getSteerMotorPos
     *      Return Steer Motor Position 
     * @return double Steer Motor Position (Raw sensor units)
     */
    public double getSteerMotorPos(){
        return steerMotor.getSelectedSensorPosition();
    }

    /** getSteerAngle
     *      Return Steer Motor Angle in Rotation2d 
     * @return Rotation2d Steer Motor Angle
     */
    public Rotation2d getSteerAngle(){
        double steerAngleRADS = Units.degreesToRadians(getSteerMotorPos()*360/4096);
        Rotation2d steerAngle = new Rotation2d(steerAngleRADS);
        return steerAngle;
    }
}
