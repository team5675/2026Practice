package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

        public static final Pose2d BLUE_TOWER_OUTPOST = new Pose2d(1.247,5.430,new Rotation2d(90));
        public static final Pose2d BLUE_TOWER_NOT_OUTPOST = new Pose2d(1.247, 1.744, new Rotation2d(270));

    }
    public class IndexerConstants {
        public static final int indexerMotorId = 32;
        
    }
    public class ClimberConstants {
        public static final int climberMotorId = 20;
        
    }
    public class IntakeConstants {
        public static final int intakeMotorId = 30;
        public static final int extendMotorId = 31;
    }
    public class ShooterConstants {
        public static final int flyWheelMotorId = 15; //left
        public static final int followerMotorId = 16;
        public static final int providerMotorId = 17;
        public static final int hoodMotorId = 18;
    }
}