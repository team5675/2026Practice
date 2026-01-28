// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer.Indexer;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  public SparkMax climberMotor;
  public SparkClosedLoopController pidController;
  public RelativeEncoder ticksEncoder;
  public Climber() {
    climberMotor = new SparkMax(Constants.ClimberConstants.climberMotorId, MotorType.kBrushless);
    ticksEncoder = climberMotor.getEncoder();
    pidController = climberMotor.getClosedLoopController();
  }

  public void setTarget(int level) {
    pidController.setSetpoint(level, ControlType.kPosition);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public static Climber instance;
  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }
}
