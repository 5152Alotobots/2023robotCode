// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Library.MotorControllers.TalonFX;

/** Add your docs here. */
public class TalonFX_Conversions {

    public static final double TALONFX_CNTS_TO_REVS = 0.00048828125;   // 1/2048
    public static final double REVS_TO_TALONFX_CNTS = 2048;


    /** talonFXToDegrees
     * @param counts TalonFX Counts
     * @return Degrees of Rotation of Mechanism
     */
    //public static double talonFXToDegrees(double counts) {
    //    // counts to degrees => 360/4096 = 0.087890625
    //    return counts * 0.087890625;
    //}

    /** degreesToTalonFX
     * @param degrees Degrees of rotation of Mechanism
     * @return TalonFX Counts
     */
    public static double degreesToTalonFX(double degrees) {
        // degrees to counts => 4096/360 = 11.37778
        double counts =  degrees * 11.37778;
        return counts;
    }

    /** talonFXToDegrees_wGearRatio
     * @param counts TalonFX Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    //public static double talonFXToDegrees_wGearRatio(double counts, double gearRatio) {
    //    // counts to degrees => 360/4096 = 0.087890625
    //    return counts * 0.087890625 / gearRatio;
    //}

    /** degreesToTalonFX_wGearRatio
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return TalonFX Counts
     */
    //public static double degreesToTalonFX_wGearRatio(double degrees, double gearRatio) {
    //    // degrees to counts => 4096/360 = 11.37778
    //    double counts =  degrees * 11.37778 / gearRatio;
    //    return counts;
    //}

    /** talonFXToRevs
     *   Convert from TalonFX Sensor Counts to Revolutions
     * @param counts double TalonFX Sensor Counts
     * @return double Revolutions
     */
    public static double talonFXToRevs (double counts){
        return counts*TALONFX_CNTS_TO_REVS;
    }

    /** talonFXToRevs_GearRatio
    *     Convert from TalonFX Counts to Revolutions through a gearbox
    * @param counts double TalonFX Sensor Counts
    * @param gearRatio double GearRatio
    * @return double Revolutions
    */
    public static double talonFXToRevs_GearRatio (double counts, double gearRatio){
        return counts*TALONFX_CNTS_TO_REVS*gearRatio;
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }
}
