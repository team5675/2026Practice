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
  public SparkMax flywheelMotor;
  public SparkMax hoodMotor;
  public SparkMax providerMotor;
  public boolean isFlywheelActive = false;
  public boolean isProviderActive = false;
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

  public void target() {
    //rotate robot and hood into position
    
  }

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
