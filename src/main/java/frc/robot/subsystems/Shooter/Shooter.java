// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public SparkMax shooterMotor;
  public boolean isShooterActive = false;
  public Shooter() {
    shooterMotor = new SparkMax(Constants.ShooterConstants.shooterMotorId, MotorType.kBrushless);
  }

  public void startShooting() {
    if (isShooterActive = false) {
      shooterMotor.set(1);
      isShooterActive = true;
    }
  }

  public void stopShooting() {
    shooterMotor.set(0);
    isShooterActive = false;
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
