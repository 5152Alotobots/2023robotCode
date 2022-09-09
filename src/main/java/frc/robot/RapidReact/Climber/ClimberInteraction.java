// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftConstants.ClimberLiftEncoder;
import frc.robot.RapidReact.Climber.ClimberRotator.ClimberRotatorConstants.ClimberRotatorExternalEncoder;

/** Add your docs here. */
public class ClimberInteraction {
    private double m_ClimberRotatorPos = 0; // Deg
    private double m_ClimberLiftPos = 0; // mm
    private double m_LiftRotateOffset = 0; // mm
    private double m_LiftRotateMaxPos = 360;  // Deg
    private double m_LiftRotateMinPos = -360; // Deg

    public void setRotatorPos(double climberRotatorPos){
        m_ClimberRotatorPos = climberRotatorPos;
        SmartDashboard.putNumber("CI_RotatorPos", m_ClimberRotatorPos);
        calcOffset();
    }

    public void setLiftPos(double climberLiftPos){
        m_ClimberLiftPos = climberLiftPos;
        SmartDashboard.putNumber("CI_LiftPos", m_ClimberLiftPos);
    }

    public double getRotatorPos(){
        return m_ClimberRotatorPos;
    }

    public double getLiftPos(){
        return m_ClimberLiftPos;
    }

    public double getLiftRotateOffset(){
        return m_LiftRotateOffset;
    }

    public double getRotatorMaxPos(){
        calcRotateMaxBound();
        return m_LiftRotateMaxPos;
    }

    public double getRotatorMinPos(){
        calcRotateMinBound();
        return m_LiftRotateMinPos;
    }

    private void calcOffset(){
        m_LiftRotateOffset = 
        (ClimberRotatorExternalEncoder.kInitialSensorPos-m_ClimberRotatorPos)
        *ClimberConstants.ClimberLiftRotator.kRotateDeg2Liftmm;
        SmartDashboard.putNumber("CI_LiftRotateOffset", m_LiftRotateOffset);
    }

    private void calcRotateMaxBound(){
        // --- Max
        // Positive Rotation is Negative Lift Movement

        // Distance from Current Position to Min
        double maxAvailTravel = m_ClimberLiftPos - ClimberLiftEncoder.kMinClimberLiftPos + m_LiftRotateOffset;
        SmartDashboard.putNumber("CI_maxAvailTravel", maxAvailTravel);
        // Calculate Max Available Rotation        
        m_LiftRotateMaxPos = m_ClimberRotatorPos + maxAvailTravel/ClimberConstants.ClimberLiftRotator.kRotateDeg2Liftmm;
        SmartDashboard.putNumber("CI_LiftRotateMaxPos", m_LiftRotateMaxPos);
    }

    private void calcRotateMinBound(){
        // --- Min
        // Negative Rotation is Positive Lift Movement
        
        // Distance from Current Position to Max
        double minAvailTravel = ClimberLiftEncoder.kMaxClimberLiftPos - m_ClimberLiftPos + m_LiftRotateOffset;
        SmartDashboard.putNumber("CI_minAvailTravel", minAvailTravel);
        // Calculate Min Available Rotation
        m_LiftRotateMinPos = m_ClimberRotatorPos - minAvailTravel/ClimberConstants.ClimberLiftRotator.kRotateDeg2Liftmm;
        SmartDashboard.putNumber("CI_LiftRotateMinPos", m_LiftRotateMinPos);
    }
}
