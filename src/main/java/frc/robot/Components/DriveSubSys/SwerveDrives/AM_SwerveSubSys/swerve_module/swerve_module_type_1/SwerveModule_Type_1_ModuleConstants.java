/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components.DriveSubSys.SwerveDrives.AM_SwerveSubSys.swerve_module.swerve_module_type_1;

/**
 * Add your docs here.
 */


public class SwerveModule_Type_1_ModuleConstants {

    // Swerve Module String ID (FL, FR, RL, RR)
    public String m_ServeModuleStringID;

    // Drive Motor Constants
    public boolean m_DrvMtr_Inverted = false;
    public boolean m_DrvMtr_SensorPhaseInverted = false;

    // Steer Motor Constants
    public boolean m_StrMtr_Inverted = false;
    public boolean m_StrMtr_SensorPhaseInverted = false;
    public int m_StrMtr_SensorZeroPosCnts = 0;

    // Enable Shuffleboard
    public boolean m_SwModule_ShuffleBoard_Enable = false;

    /**
    * Constructs a SwerveModule_Type_1.
    *
    * @param swerveModuleStringID        String Swerve Module String ID (FL,FR,RL,RR) 
    * @param drvMtr_Inverted             boolean Drive Motor Inverted
    * @param drvMtr_SensorPhaseInverted  boolean Drive Motor Sensor Phase Inverted
    * @param strMtr_Inverted             boolean Steer Motor Inverted
    * @param strMtr_SensorPhaseInvered   boolean Steer Motor Sensor Phase Inverted
    * @param strMtr_SensorZeroPosCnts    int Steer Motor Sensor Zero Postion Counts
    * @param enable_Shuffleboard         boolean Enable Shuffleboard for this module
    */
    public SwerveModule_Type_1_ModuleConstants(
        String swerveModuleStringID,
        boolean drvMtr_Inverted,
        boolean drvMtr_SensorPhaseInverted,
        boolean strMtr_Inverted,
        boolean strMtr_SensorPhaseInvered,
        int strMtr_SensorZeroPosCnts,
        boolean swModule_Shuffleboard_Enable)
    {
        m_ServeModuleStringID = swerveModuleStringID;
        m_DrvMtr_Inverted = drvMtr_Inverted;
        m_DrvMtr_SensorPhaseInverted = drvMtr_SensorPhaseInverted;
        m_StrMtr_Inverted = strMtr_Inverted;
        m_StrMtr_SensorPhaseInverted = strMtr_SensorPhaseInvered;
        m_StrMtr_SensorZeroPosCnts = strMtr_SensorZeroPosCnts;
        m_SwModule_ShuffleBoard_Enable = swModule_Shuffleboard_Enable;
    }
}
