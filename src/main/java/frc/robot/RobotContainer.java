// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.invoke.LambdaConversionException;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Climber.ClimbCommand;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.LowerClimbCommand;
import frc.robot.subsystems.Climber.RaiseClimbCommand;
//import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Shooter.Shooter;

public class RobotContainer {

    public double testAutomaticRotation(double tx,double driverRot){
        
    //    double robotRotation = drivetrain.getPigeon2().getRotation2d().getDegrees();
    //    System.out.println(robotRotation);


    //     double neededRotation = tx + robotRotation;

    //     double diff = robotRotation - neededRotation;

        if (Math.abs(driverRot) > 0.1){
            return driverRot;
        }else{
            return -tx*0.03;
        }  
    }

        public double ShooterAngle(double targetRotationFinal, double ShootRot){
            if(Math.abs(ShootRot) > 0.1){
                return ShootRot;
            }
            else{
                return -targetRotationFinal*0.03;
            }
    }

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    

 /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public Command pathfindingCommand;


    public RobotContainer() {
        NamedCommands.registerCommand("RaiseClimber", new RaiseClimbCommand());
        NamedCommands.registerCommand("Climber", new ClimbCommand());

        autoChooser = AutoBuilder.buildAutoChooser("Moveshootaroundmap_leah.auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {

             drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        ); } else {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        ); }
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //             // Removed negative to reverse rotation on map
        //     )
        // );

        //driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));
        driverController.leftStick().onTrue(
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(testAutomaticRotation(LimelightHelpers.getTX("limelight-hailo"),-driverController.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )

        //Commands.run(()-> System.out.println(LimelightHelpers.getTX("limelight-hailo")))
    );
        driverController.rightStick().onTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(Commands.runOnce(() -> drivetrain.setOperatorPerspectiveForward(DriverStation.getAlliance().get().equals(Alliance.Red) ? Rotation2d.k180deg : Rotation2d.kZero)));

        //driverController.rightTrigger().whileTrue(Commands.run(() -> Shooter.getInstance().startShooting()));
        //driverController.rightTrigger().onFalse(Commands.run(() -> Shooter.getInstance().stopShooting()));
        // driverController.povUp().whileTrue(Commands.runOnce(()->Climber.getInstance().climberMotor.set(1)));
        // //driverController.povUp().onFalse(Commands.run(()->Climber.getInstance().climberMotor.set(0)));
        // driverController.povDown()
        // .whileTrue(Commands.runOnce(()->Climber.getInstance().climberMotor.set(-1)))
        // .and(driverController.povUp())
        // .onFalse(Commands.runOnce(()->Climber.getInstance().climberMotor.set(0)));

        // pov UP motor moves up
        // let go and stop

        // pov DOWN motor goes down
        // let go and stop

        //lower
        driverController.povDown().onTrue(Commands.runOnce(() -> {Climber.getInstance().climberMotor.set(1);}));
        //raise
        driverController.povUp().onTrue(Commands.runOnce(() -> {Climber.getInstance().climberMotor.set(-1);}));

        driverController.x().onTrue(Commands.runOnce(() -> {Climber.getInstance().climberMotor.set(0);}));

        driverController.y().onTrue(Commands.runOnce(() -> {Climber.getInstance().ticksEncoder.setPosition(0);}));
    
        // driverController.rightTrigger().onTrue(new LowerClimbCommand());
        // driverController.leftTrigger().onTrue(new RaiseClimbCommand());
        // driverController.a().onTrue(new ClimbCommand());

    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}