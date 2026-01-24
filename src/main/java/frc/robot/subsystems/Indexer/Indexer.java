// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  //Make one motor
  //Runs motor
  //Stop motor
  public SparkMax motor;
  public Indexer() {
    motor = new SparkMax(Constants.IndexerConstants.motorid, MotorType.kBrushless);
  }
  @Override
  public void periodic() {  }

  public void startindexer() {
    motor.set(1);
  }
  public void stopindexer() {
    motor.set(0);
  }

  public static Indexer instance;
  public static Indexer getInstance() {
    if (instance == null) {
      instance = new Indexer();
    }
    return instance;
  }
} 
