package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public class DTPshi {
    public static Pose2d findClosestLowCave(Pose2d currentPose){
        Pose2d closestPose = new Pose2d();
        double closestDist = Double.MAX_VALUE;

        for (Pose2d pose : Constants.FieldConstants.ArrLowCaveDTPPoses){
            double curDist = currentPose.getTranslation().getDistance(pose.getTranslation());
            if (curDist < closestDist){
                closestPose = pose;
                closestDist = curDist; // <-- the missing line
            }
        }

        return closestPose;
    }


    public static Pose2d findClosestHighCave(Pose2d currentPose){
        Pose2d closestPose = new Pose2d();
        double closestDist = Double.MAX_VALUE;

        for (Pose2d pose : Constants.FieldConstants.ArrHighCaveDTPPoses){
            double curDist = currentPose.getTranslation().getDistance(pose.getTranslation());
            if (curDist < closestDist){
                closestPose = pose;
                closestDist = curDist; // <-- the missing line
            }
        }

        return closestPose;
    }
}
