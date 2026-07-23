package frc.robot.AutoStuff;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class BlineStuff {

    private BlineStuff() {}

    /**
     * Builds every registered BLine auto up front, keyed by name, so the auto
     * chooser can pick by String and getAutonomousCommand is a simple lookup —
     * no re-building against the shared, stateful pathBuilder at match time.
     */
    public static Map<String, Command> buildAutos(FollowPath.Builder pathBuilder, CommandSwerveDrivetrain drivetrain) {
        Map<String, Command> autos = new LinkedHashMap<>(); // LinkedHashMap keeps insertion order for the chooser UI

        // --- First Straight ---
        Path firstStraight = new Path("first-straight");
        Command firstAuto = pathBuilder
                .withPoseReset(drivetrain::resetPose)
                .build(firstStraight);
        pathBuilder.withPoseReset(ignored -> {}); // clear stateful option before the next build
        autos.put("First Straight", firstAuto);

        // --- add more autos here, following the same pattern ---
        // Path secondPath = new Path("second-path");
        // Command secondAuto = pathBuilder
        //         .withPoseReset(drivetrain::resetPose)
        //         .build(secondPath);
        // pathBuilder.withPoseReset(ignored -> {});
        // autos.put("Second Auto", secondAuto);

        return autos;
    }
}