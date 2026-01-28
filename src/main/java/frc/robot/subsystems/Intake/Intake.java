// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer.Indexer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public SparkMax intakeMotor;
  public SparkMax extendMotor;
  public Intake() {
    intakeMotor = new SparkMax(Constants.IntakeConstants.intakeMotorId, MotorType.kBrushless);
    extendMotor = new SparkMax(Constants.IntakeConstants.extendMotorId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    
  }
  public void extendIntake() {
    extendMotor.set(1);
    // needs to stop after the intake is fully extended
  }
  public void retractIntake() {
    extendMotor.set(-1);
    // needs to stop after the intake is fully retracted
  }
  public void startIntake() {
    intakeMotor.set(1);
  }
  public void stopIntake() {
    intakeMotor.set(0);
  }

  public static Intake instance;
  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }
}
