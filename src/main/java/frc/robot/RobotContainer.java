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
import frc.robot.subsystems.Aim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Climber.ClimbCommand;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.LowerClimbCommand;
import frc.robot.subsystems.Climber.RaiseClimbCommand;
//import frc.robot.subsystems.FuelDetection.FuelDetection;
import frc.robot.subsystems.Shooter.Shooter;

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

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    private int flip;

 /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public Command pathfindingCommand;

    public boolean isRedAlliance(){
            return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }


    public RobotContainer() {
        NamedCommands.registerCommand("RaiseClimber", new RaiseClimbCommand());
        NamedCommands.registerCommand("Climber", new ClimbCommand());

        autoChooser = AutoBuilder.buildAutoChooser("Moveshootaroundmap_leah.auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {

             drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {

                flip = isRedAlliance() ? -1 : 1;

                return drive.withVelocityX(flip*driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(flip*driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
             
             }
            )
        ); 

        //driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        // ));
       

        //Commands.run(()-> System.out.println(LimelightHelpers.getTX("limelight-hailo")))
    //);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(Commands.runOnce(() -> drivetrain.setOperatorPerspectiveForward(isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero)));


        aimAtHub.HeadingController.setPID(8,0,0.005);
        aimAtHub.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        driverController.a().whileTrue(
            drivetrain.applyRequest(() -> {
                flip = isRedAlliance() ? -1 : 1;

                return aimAtHub.withVelocityX(flip*driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(flip*driverController.getLeftX() * MaxSpeed)
                    .withTargetDirection(Aim.getInstance(drivetrain).angleToHub);
            }
        ));

        //FuelDetection.getInstance();
        //driverController.rightTrigger().whileTrue(Commands.run(() -> Shooter.getInstance().startShooting()));
        //driverController.rightTrigger().onFalse(Commands.run(() -> Shooter.getInstance().stopShooting()));
        // driverController.povUp().whileTrue(Commands.runOnce(()->Climber.getInstance().climberMotor.set(1)));
        // //driverController.povUp().onFalse(Commands.run(()->Climber.getInstance().climberMotor.set(0)));
        // driverController.povDown()
        // .whileTrue(Commands.runOnce(()->Climber.getInstance().climberMotor.set(-1)))
        // .and(driverController.povUp())
        // .onFalse(Commands.runOnce(()->Climber.getInstance().climberMotor.set(0)));

        //lower
        // driverController.povDown().onTrue(Commands.runOnce(() -> {Climber.getInstance().climberMotor.set(1);}));
        // //raise
        // driverController.povUp().onTrue(Commands.runOnce(() -> {Climber.getInstance().climberMotor.set(-1);}));

        // driverController.x().onTrue(Commands.runOnce(() -> {Climber.getInstance().climberMotor.set(0);}));

        // driverController.y().onTrue(Commands.runOnce(() -> {Climber.getInstance().ticksEncoder.setPosition(0);}));
    
        // driverController.rightTrigger().onTrue(new LowerClimbCommand());
        // driverController.leftTrigger().onTrue(new RaiseClimbCommand());
        // driverController.a().onTrue(new ClimbCommand());

        // driverController.a().whileTrue(Commands.run(() -> {
       
        //   Shooter.getInstance().hoodMotor.set(0.1);
        // }));

        // driverController.a().whileFalse(Commands.run(() -> {
      
        // Shooter.getInstance().hoodMotor.set(0);
        // }));
        // driverController.x().whileTrue(Commands.run(() -> {
        //     Shooter.getInstance().setFlywheelRPM(2800);
        //     Shooter.getInstance().runFeederMotor();
    
        // }));

        // driverController.x().whileFalse(Commands.run(() -> {
        //     Shooter.getInstance().stopFlywheel();
      
        // }));


    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}