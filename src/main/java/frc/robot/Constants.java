// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

/** Add your docs here. */
public class Constants {
        public static abstract class ElevatorConstants{
            public static final Angle In = Rotation.of(0);
            public static final Angle Mid = Rotation.of(3);
            public static final Angle Out = Rotation.of(6);

        }

        public static abstract class ArmConstants{
            public static final Angle Handoff = Degree.of(-105);
            public static final Angle Scoring = Degree.of(25);
            public static final Angle climb = Degree.of(90);
        }

        public static abstract class DriveConstants{
            public static final double TranslationDeadband = 0.1;
            public static final double RotationDeadband = 0.1;

            public static final double RotationalToleranceDegrees = 3.5;
            public static final double RotationalToleranceDegreesAUTO = 6.0;
        }
}