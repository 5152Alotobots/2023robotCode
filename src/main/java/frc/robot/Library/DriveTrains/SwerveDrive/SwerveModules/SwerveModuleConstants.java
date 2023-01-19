// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Library.DriveTrains.SwerveDrive.SwerveModules;

/** Add your docs here. */
public class SwerveModuleConstants {
    public final String moduleName;
    public final int driveMotorID;
    public final boolean driveMotorInvert;
    public final int steerMotorID;
    public final boolean steerMotorInvert;
    public final int cancoderID;
    public final double angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param moduleName
     * @param driveMotorID
     * @param driveMotorInvert
     * @param steerMotorID
     * @param steerMotorInvert
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(
        String moduleName,
        int driveMotorID,
        boolean driveMotorInvert,
        int steerMotorID,
        boolean steerMotorInvert,
        int canCoderID,
        double angleOffset){
        
            this.moduleName = moduleName;
            this.driveMotorID = driveMotorID;
            this.driveMotorInvert = driveMotorInvert;
            this.steerMotorID = steerMotorID;
            this.steerMotorInvert = steerMotorInvert;
            this.cancoderID = canCoderID;
            this.angleOffset = angleOffset;
    }
}
