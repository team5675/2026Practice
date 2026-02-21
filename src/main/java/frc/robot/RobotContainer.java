package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Aim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Climber.ClimbCommand;
import frc.robot.subsystems.Climber.RaiseClimbCommand;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.FieldCentricFacingAngle aimAtHub = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);

    public static CommandSwerveDrivetrain drivetrain;
    public static CommandSwerveDrivetrain getDrivetrain(){
        if (drivetrain == null){
            drivetrain = TunerConstants.createDrivetrain();
        }
        return drivetrain;
    }
    
    private int flip;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public Command pathfindingCommand;

    public boolean isRedAlliance(){
            return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    public RobotContainer() {
        RobotContainer.getDrivetrain();
        
        NamedCommands.registerCommand("RaiseClimber", new RaiseClimbCommand());
        NamedCommands.registerCommand("Climber", new ClimbCommand());

        autoChooser = AutoBuilder.buildAutoChooser("Moveshootaroundmap_leah.auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        

             drivetrain.setDefaultCommand(
            
            drivetrain.applyRequest(() -> {

                //Flip negation if red or blue
                flip = isRedAlliance() ? -1 : 1;

                return drive.withVelocityX(flip*driverController.getLeftY() * MaxSpeed) 
                    .withVelocityY(flip*driverController.getLeftX() * MaxSpeed) 
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate); 
             
             }
            )
        ); 

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(Commands.runOnce(() -> drivetrain.setOperatorPerspectiveForward(isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero)));

        // Aim at Hub      
        aimAtHub.HeadingController.setPID(8,0,0.005);
        aimAtHub.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        driverController.a().whileTrue(
            drivetrain.applyRequest(() -> {
                flip = isRedAlliance() ? -1 : 1;

                return aimAtHub.withVelocityX(flip*driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(flip*driverController.getLeftX() * MaxSpeed)
                    .withTargetDirection(Aim.getInstance().angleToHub);
            }
        ));
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}