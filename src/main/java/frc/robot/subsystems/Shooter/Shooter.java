// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public SparkMax flywheelMotor;
  public SparkMax hoodMotor;
  public SparkMax providerMotor;
  public boolean isFlywheelActive = false;
  public boolean isProviderActive = false;
  public double distanceToHub;
  public Pose2d hubPose;
  
  /* public static double getTurnSpeedToTarget(Pose2d robotPose, Translation2d targetPos, double kP) {   
     // Compute direction vector from robot to target   
      Translation2d delta = targetPos.minus(robotPose.getTranslation());       
       // Desired heading (angle of the direction vector)   
        Rotation2d desiredHeading = delta.getAngle();     
           // Angular error (robot heading vs. desired), normalized to [-180, 180] degrees  
        double angleError = desiredHeading.minus(robotPose.getRotation()).getDegrees();    
        angleError = MathUtil.angleModulus(angleError);  
        // Ensures [-180, 180]       
         // Proportional control output, clamped to [-1, 1]    
        return MathUtil.clamp(angleError * kP, -1.0, 1.0);}*/
        
  public Shooter() {
    flywheelMotor = new SparkMax(Constants.ShooterConstants.flyWheelMotorId, MotorType.kBrushless);
    hoodMotor = new SparkMax(Constants.ShooterConstants.hoodMotorId, MotorType.kBrushless);
    providerMotor = new SparkMax(Constants.ShooterConstants.providerMotorId, MotorType.kBrushless);
  }

  /* 
     change controls for shooter later one for targeting, 
     one for activate/deactivate and one for shooting
  */ 

  public void activateShooter() {
    //activate shooter at the start of the match
    if (isFlywheelActive = false) {
      flywheelMotor.set(1);
      isFlywheelActive = true;
      //change to a non-lethal speed
    }
  }

  public double hoodCalc(double distance) {
    distance = distanceToHub;
    double a = 1.0;
    double b = 1.0;
    double c = 1.0;
    double d = 1.0;

    return a * (distance * distance * distance) + b * (distance * distance) + c * distance + d;
  }

  public double rpmCalc(double distance) {
    distance = distanceToHub;
    double a = 1.0;
    double b = 1.0;
    double c = 1.0;
    double d = 1.0;
    return a * (distance * distance * distance) + b * (distance * distance) + c * distance + d;
  }

  // the x value of the red hub is 468.565
  // the y value of the red hub is 205.84
  // distance formula and stuff

  

    public void target(CommandSwerveDrivetrain drivetrain) {

      hubPose = new Pose2d(4.625, 4.035, new Rotation2d(0));
    Pose2d robotPose = drivetrain.getState().Pose;
    //get distance
    double distanceToHub = robotPose.getTranslation().getDistance(hubPose.getTranslation());
    double theHood = hoodCalc(distanceToHub);
    double targetRotationX = hubPose.getX() - robotPose.getX();
    double targetRotationY = hubPose.getY() - robotPose.getY();
    double targetRotationFinal = Math.atan(targetRotationY/targetRotationX);
    
  }

  // the hub (4.625, 4.035, 0);
  

  public void startShooting() {
   if (isProviderActive = false) {
      providerMotor.set(1);
      isProviderActive = true;
   }
  }

  public void stopShooting() {
    providerMotor.set(0);
    isProviderActive = false;
  }
  

  public void deactivateShooter() {
    flywheelMotor.set(0);
    isFlywheelActive = false;
  }

  @Override
  public void periodic() {
    
  }

  public static Shooter instance;
  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }
}
