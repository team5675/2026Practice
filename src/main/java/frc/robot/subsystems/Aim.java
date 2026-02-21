// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Aim extends SubsystemBase {
  private CommandSwerveDrivetrain drivetrain;
  public Rotation2d angleToHub;
  public Translation2d diffToHub;
  public double distanceToHub;

  public Aim() {
    angleToHub = new Rotation2d();
    diffToHub = new Translation2d();
    drivetrain = RobotContainer.getDrivetrain();
  }

  @Override
  public void periodic() {
    Pose2d robot = drivetrain.getState().Pose;
    Translation2d hub =
        (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
            ? Constants.PoseConstants.RED_HUB_CENTER
            : Constants.PoseConstants.BLUE_HUB_CENTER;
    
    double dx = hub.getX() - robot.getX();
    double dy = hub.getY() - robot.getY();

    // double neededangle = Math.atan2(dy, dx);
    // Angle angle = Angle.ofBaseUnits(neededangle, Radians);

    angleToHub = new Rotation2d(dx, dy);

    diffToHub = new Translation2d(dx, dy);

    distanceToHub = Math.hypot(dx,dy);
    SmartDashboard.putNumber("Aim/HubX", hub.getX());
    SmartDashboard.putNumber("Aim/HubY", hub.getY());
    SmartDashboard.putNumber("Aim/Hub Angle (deg)", angleToHub.getDegrees());
    SmartDashboard.putNumber("Aim/Hub Angle (rad)", angleToHub.getRadians());
    SmartDashboard.putNumber("Aim/Hub Angle (sin)", angleToHub.getSin());
    SmartDashboard.putNumber("Aim/Hub Angle (cos)", angleToHub.getCos());
    SmartDashboard.putNumber("Aim/Robot Heading (deg)", drivetrain.getState().Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Aim/Robot Heading (rad)", drivetrain.getState().Pose.getRotation().getRadians());
    SmartDashboard.putNumber("Aim/dx", dx);
    SmartDashboard.putNumber("Aim/dy", dy);
  }

  private static Aim instance;
  public static Aim getInstance() {
    if (instance == null) {
      instance = new Aim();
    }
    return instance;
  }
}
