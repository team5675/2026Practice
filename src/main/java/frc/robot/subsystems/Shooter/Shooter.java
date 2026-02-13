// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
  public TalonFX flywheelMotor;
  public TalonFX followerMotor;
  public TalonFX hoodMotor;
  public TalonFX providerMotor;
  public boolean isFlywheelActive = false;
  public boolean isProviderActive = false;
  public double distanceToHub;
  public Pose2d hubPose;
  public Follower follower;
        
  public Shooter() {
    flywheelMotor = new TalonFX(Constants.ShooterConstants.flyWheelMotorId, "Default Name");
    followerMotor = new TalonFX(Constants.ShooterConstants.followerMotorId, "Default Name");
    hoodMotor = new TalonFX(Constants.ShooterConstants.hoodMotorId, "Default Name");
    providerMotor = new TalonFX(Constants.ShooterConstants.providerMotorId, "Default Name");

    follower = new Follower(flywheelMotor.getDeviceID(),  MotorAlignmentValue.Aligned);
    followerMotor.setControl(follower);
  }

  public void activateShooter() {
    //activate shooter at the start of the match
    if (isFlywheelActive = false) {
      flywheelMotor.set(0.2);
      isFlywheelActive = true;
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

  // blue hub (4.625, 4.0346, 0);
  // red hub (11.916, 4.0346, 0);

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

  public void setFlywheelRPM(double rpm){
    VelocityVoltage rpmSpeed = new VelocityVoltage(rpm / 60.0);
    //flywheelMotor.setControl(rpmSpeed);
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
