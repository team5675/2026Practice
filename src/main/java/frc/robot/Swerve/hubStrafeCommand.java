// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class hubStrafeCommand extends Command {

  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.FieldCentric drive;
  CommandXboxController driverController;
  double MaxSpeed;
  double MaxAngularRate;
  Pose2d hubPose;
  double rotateToHubSpeed;
  double scalar;

  /** Creates a new hubStrafeCommand. */
  public 
  hubStrafeCommand(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, CommandXboxController driverController, double MaxSpeed, double MaxAngularRate) {
    this.drivetrain = drivetrain;
    this.drive = drive;
    this.driverController = driverController;
    this.MaxSpeed = MaxSpeed;
    this.MaxAngularRate = MaxAngularRate;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
try{
    
    Pose2d robotPose = drivetrain.getState().Pose;
    System.out.println("Robot Pose " + robotPose);
    //get distance
    //double distanceToHub = robotPose.getTranslation().getDistance(hubPose.getTranslation());
    //double distanc

    // double ToHood = hoodCalc(distanceToHub);
    double dx = robotPose.getX() - hubPose.getX();
    SmartDashboard.putNumber("Target X ", dx);
    double dy = hubPose.getY() - robotPose.getY();
    SmartDashboard.putNumber("Target Y ", dy);
    double neededTheta = Math.atan2(dx, dy); //in radians
    SmartDashboard.putNumber("Target Rotation ", neededTheta);

    double rotDiff = robotPose.getRotation().getRadians() - neededTheta;
    SmartDashboard.putNumber("Rotation Difference", rotDiff);

    if(Math.abs(rotDiff) < 0.03) rotDiff = 0;

    scalar = 0.1;
    
    rotateToHubSpeed = rotDiff * scalar * MaxAngularRate;
    SmartDashboard.putNumber("Angular Velocity", rotateToHubSpeed);

    CommandScheduler.getInstance().schedule(drivetrain.applyRequest(() ->
      drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(rotateToHubSpeed)
    ));
    
  
    }catch(Exception e) {
      e.printStackTrace();
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
