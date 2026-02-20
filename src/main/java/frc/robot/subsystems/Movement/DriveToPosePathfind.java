package frc.robot.subsystems.Movement;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class DriveToPosePathfind {
    public static Command createFlipped(Pose2d targetPose) {
        return AutoBuilder.pathfindToPoseFlipped(
            targetPose,
            Constants.PathplannerConstants.constraints,
            0.0   // goalEndVelocity: stop when we arrive (m/s)
        );
    }

    public static Command create(Pose2d targetPose) {
        return AutoBuilder.pathfindToPose(
            targetPose,
            Constants.PathplannerConstants.constraints,
            0.0   // goalEndVelocity: stop when we arrive (m/s)
        );
    }
}
