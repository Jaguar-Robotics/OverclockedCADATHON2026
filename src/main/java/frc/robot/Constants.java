// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class Constants {
        public static abstract class ElevatorConstants{
            public static final Distance In = Inches.of(0);
            public static final Distance Mid = Inches.of(20);
            public static final Distance Out = Inches.of(43.5);

        }

        public static abstract class ArmConstants{
            public static final Angle Handoff = Degree.of(-105);
            public static final Angle Scoring = Degree.of(25);
            public static final Angle climb = Degree.of(90);

            public static final Transform3d GemstoneOffset = new Transform3d(
            new Translation3d(Inches.of(16), Inches.of(0), Inches.of(-4)),
            new Rotation3d(Degrees.of(0), Degrees.of(30), Degrees.of(0)) // assuming the 30° is a pitch offset, same axis convention as your other rotations — confirm below
        );
        }

        public static abstract class DriveConstants{
            public static final double TranslationDeadband = 0.1;
            public static final double RotationDeadband = 0.1;

            public static final double RotationalToleranceDegrees = 3.5;
            public static final double RotationalToleranceDegreesAUTO = 6.0;
        }

        public static abstract class FieldConstants{
                public static final Pose2d[] ArrLowCaveDTPPoses = {
                    new Pose2d(11.011, 3.554, Rotation2d.fromDegrees(22.3)), //bot left
                    new Pose2d(11.793, 2.768 , Rotation2d.fromDegrees(67.77)), //mid bot left
                    new Pose2d(12.934, 2.771 , Rotation2d.fromDegrees(112.2)), //mid high left
                    new Pose2d(13.706, 3.556 , Rotation2d.fromDegrees(157.56)), //high left
                    new Pose2d(13.711, 4.668, Rotation2d.fromDegrees(-157.56)), //bot right
                    new Pose2d(12.920, 5.465 , Rotation2d.fromDegrees(-112.2)) //mid bot right
                };
        }
}