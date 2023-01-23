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
import frc.robot.Constants;
import frc.robot.Library.DriveTrains.SwerveDrive.SwerveModules.CTREModuleState;
import frc.robot.Library.DriveTrains.SwerveDrive.SwerveModules.SwerveModuleConstants;
import frc.robot.Library.DriveTrains.SwerveDrive.SwerveModules.MK4i_FalconFalcon.MK4i_FalconFalcon_Module_Constants.*;
import frc.robot.Library.MotorControllers.TalonFX.TalonFX_Conversions;
import frc.robot.Library.MotorControllers.TalonSRX.TalonSRX_Conversions;

/** Add your docs here. */
public class MK4i_FalconFalcon_Module {

    public String moduleName;
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANCoder steerAngleEncoder;
    private double strAngleOffset;  
    private double lastAngle;

    //public int moduleNumber;

    SimpleMotorFeedforward driveMotorFF = new SimpleMotorFeedforward(
        DriveMotor.driveKS,
        DriveMotor.driveKV,
        DriveMotor.driveKA);

    public MK4i_FalconFalcon_Module(
        String moduleName,
        SwerveModuleConstants moduleConstants)
        {
            this.moduleName = moduleName;

            /* Drive Motor Config */
            this.driveMotor = new TalonFX(moduleConstants.driveMotorID);
            configDriveMotor(moduleConstants);         

            /* Angle Encoder Config */
            this.steerAngleEncoder = new CANCoder(moduleConstants.cancoderID);
            configSteerAngleEncoder(moduleConstants);

            /* Steer Motor Config */
            this.steerMotor = new TalonFX(moduleConstants.steerMotorID);
            configSteerMotor(moduleConstants);

            this.strAngleOffset = moduleConstants.angleOffset;
        
            lastAngle = getState().angle.getDegrees();
        }

    /* ***** Swerve Module Methods ***** */

    public void setDesiredState(
        SwerveModuleState desiredState,
        boolean isOpenLoop)
        {
            desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

            /** Drive Motor Command */
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

            /** Steer Motor Command
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
                TalonFX_Conversions.degreesToCANCoder(
                    angle,
                    1.0));
            // Update last angle         
            lastAngle = angle;
        }

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
        double drvDistance;
        Rotation2d strAngle;

        drvDistance = driveMotor.getSelectedSensorPosition();
        strAngle = getSteerAngle();

    swrModulePosition = new SwerveModulePosition(drvDistance, strAngle);

    return swrModulePosition;
  }

    /* ***** Drive Motor Methods ***** */  
  
    private void configDriveMotor(SwerveModuleConstants moduleConstants){        
        driveMotor.configFactoryDefault();
        //driveMotor.configAllSettings(MK4i_FalconFalcon_Module_Constants.DriveTalonFXConfig);
        driveMotor.setInverted(moduleConstants.driveMotorInvert);
        driveMotor.setNeutralMode(MK4i_FalconFalcon_Module_Constants.DriveMotor.neutralMode);
        //driveMotor.setSelectedSensorPosition(0);
    }
    
    /* ***** Steer Angle Encoder Methods ***** */

    /**  configSteerAngleEncoder
     *      Configure Steer Angle Encoder
     * @param moduleConstants
     */
    private void configSteerAngleEncoder(SwerveModuleConstants moduleConstants){
        steerAngleEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        steerAngleEncoder.configMagnetOffset(moduleConstants.angleOffset);
        steerAngleEncoder.configSensorDirection(moduleConstants.SteerCANcoderInvert);
    }
    
    /** getSteerSensorAbsolutePos
     *   Returns the Absolute Position Measurement
     * @return double Absolute Steer Sensor Positions Measurement  
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

    /* ***** Steer Motor Methods ***** */
    
    /** configSteerMotor
     * 
     * @param moduleConstants
     */
    private void configSteerMotor(SwerveModuleConstants moduleConstants){
        steerMotor.configFactoryDefault();
        //steerMotor.configAllSettings(MK4i_FalconFalcon_Module_Constants.SteerTalonFXConfig);
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
    }

    /** getSteerMotorPos
     *      Return Steer Motor Position (Need to specify PID?)
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
        return new Rotation2d();
    }
}
