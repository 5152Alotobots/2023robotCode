// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RapidReact.Climber;

import frc.robot.RapidReact.Climber.ClimberLift.ClimberLiftConstants.ClimberLiftEncoder;

/** Add your docs here. */
public class ClimberConstants {
    public final class ClimberLiftRotator{
        public static final double kRotateDeg2Liftmm =
        4090.0*ClimberLiftEncoder.kFeedbackCoefficient/360.0; 
        // 4090 cnts per rev
        // 4090*515/20706 = 101.7 mm per rev
        // (4090*515/20706)/360 = 0.28257376 mm per degree
      }
}