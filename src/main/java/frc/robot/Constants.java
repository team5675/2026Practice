package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    public class LimelightConstants {
        public static String limelightName = "limelight-hailo";
        public static double kTolerance = 0.2;
        public static double minStrafe = 0.3;

        public static String llHalio = "limelight-hailo";
        public static String llWide = "limelight-wide";
        public static String llCoral = "limelight-coral";
        public static String llBack = "limelight-back";
    }
    
    public class PathplannerConstants {
        // Create the constraints to use while pathfinding
        public static final PathConstraints constraints = new PathConstraints(
        2.0, 2.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540));
    }
    public class PoseConstants {
        // Values are in Meters
        public static final double BLUE_TOWER_X = 1.065;
        public static final double RED_TOWER_X = 15.48;

        public static final double BLUE_TOWER_OUTPOST_Y = 4.700;
        public static final double RED_TOWER_OUTPOST_Y = 5.265;

        public static final double BLUE_TOWER_DEPOT_Y = 4.700;
        public static final double RED_TOWER_DEPOT_Y = 3.4; 

        public static final double BLUE_STAGING_OFFSET = 0.5;
        public static final double RED_STAGING_OFFSET = 0.5;

        public static final double DEPO_DIFFERENCE = 0.1;
        
        public static final Pose2d BLUE_TOWER_OUTPOST = new Pose2d(BLUE_TOWER_X, BLUE_TOWER_OUTPOST_Y, Rotation2d.fromDegrees(90));
        public static final Pose2d BLUE_TOWER_OUTPOST_STAGING = new Pose2d(BLUE_TOWER_X, BLUE_TOWER_OUTPOST_Y + BLUE_STAGING_OFFSET, Rotation2d.fromDegrees(90));

        public static final Pose2d BLUE_TOWER_DEPOT = new Pose2d(BLUE_TOWER_X, BLUE_TOWER_DEPOT_Y, Rotation2d.fromDegrees(270));
        public static final Pose2d BLUE_TOWER_DEPOT_STAGING = new Pose2d(BLUE_TOWER_X, BLUE_TOWER_OUTPOST_Y - (BLUE_STAGING_OFFSET + DEPO_DIFFERENCE), Rotation2d.fromDegrees(270));
        
        public static final Pose2d RED_TOWER_OUTPOST = new Pose2d(RED_TOWER_X, RED_TOWER_OUTPOST_Y, Rotation2d.fromDegrees(90));
        public static final Pose2d RED_TOWER_OUTPOST_STAGING = new Pose2d(RED_TOWER_X, RED_TOWER_OUTPOST_Y - RED_STAGING_OFFSET, Rotation2d.fromDegrees(90));
        
        public static final Pose2d RED_TOWER_DEPOT = new Pose2d(RED_TOWER_X, RED_TOWER_DEPOT_Y, Rotation2d.fromDegrees(270));
        public static final Pose2d RED_TOWER_DEPOT_STAGING = new Pose2d(RED_TOWER_X, RED_TOWER_DEPOT_Y + (RED_STAGING_OFFSET - DEPO_DIFFERENCE), Rotation2d.fromDegrees(270));

        
        public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.626, 4.035);
        public static final Translation2d RED_HUB_CENTER = new Translation2d(11.915, 4.035);
    }
    public class IndexerConstants {
        public static final int indexerMotorId = 0;
        
    }
    public class ClimberConstants {
        public static final int climberMotorId = 20;
        
    }
    public class IntakeConstants {
        public static final int intakeMotorId = 0;
        public static final int extendMotorId = 0;
    }
    public class ShooterConstants {
        public static final int flyWheelMotorId = 0;
        public static final int followerMotorId = 0;
        public static final int providerMotorId = 0;
        public static final int hoodMotorId = 0;
    }
}