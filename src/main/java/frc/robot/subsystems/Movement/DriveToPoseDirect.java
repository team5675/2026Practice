package frc.robot.subsystems.Movement;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;

public class DriveToPoseDirect {
    public static Command create(CommandSwerveDrivetrain drive, Pose2d targetPose) {
        return Commands.defer(() -> {
            Pose2d currentPose = drive.getState().Pose;


            // This extrapolates  
            
            // double dx = targetPose.getX() - currentPose.getX();
            // double dy = targetPose.getY() - currentPose.getY();
            // Rotation2d travelDirection = new Rotation2d(dx, dy);

            // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            //     new Pose2d(currentPose.getTranslation(), travelDirection),
            //     new Pose2d(targetPose.getTranslation(), travelDirection)
            // );

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                currentPose, targetPose
            );

            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                Constants.PathplannerConstants.constraints,
                null,  // idealStartingState: null for on-the-fly paths
                new GoalEndState(0.0, targetPose.getRotation())
            );

            path.preventFlipping = true;

            return AutoBuilder.followPath(path);
        }, Set.of(drive));
    }
}
