// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public TalonFX flywheelMotor;
  public TalonFX followerMotor;
  public SparkMax hoodMotor;
  public TalonFX feederMotor;
  public boolean isFlywheelActive = false;
  public boolean isProviderActive = false;
  public double distanceToHub;
  public Pose2d hubPose;
  public Follower follower;
  public double rps;
  public InterpolatingDoubleTreeMap lutRPM;
  public InterpolatingDoubleTreeMap lutHood;
  private final VelocityTorqueCurrentFOC flywheelVelocity;
        
  public Shooter() {
    flywheelMotor = new TalonFX(Constants.ShooterConstants.flyWheelMotorId, "canivore");
    followerMotor = new TalonFX(Constants.ShooterConstants.followerMotorId, "canivore");
    hoodMotor = new SparkMax(Constants.ShooterConstants.hoodMotorId, MotorType.kBrushless);
    feederMotor = new TalonFX(Constants.ShooterConstants.providerMotorId, "canivore");

    flywheelVelocity =
    new VelocityTorqueCurrentFOC(0); 

    TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current limits
        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Closed-loop gains (Slot 0)
        config.Slot0.kS = 8.0;  // Static friction (Amps for FOC)
        config.Slot0.kV = 0.1;  // Velocity feedforward
        config.Slot0.kA = 0.0;  // Acceleration feedforward
        config.Slot0.kP = 12.0;   // Proportional
        config.Slot0.kI = 0.0;   // Integral (leave at 0 for flywheels!)
        config.Slot0.kD = 0.04;   // Derivative

        flywheelMotor.getConfigurator().apply(config);  
        
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        followerMotor.getConfigurator().apply(config);

    follower = new Follower(flywheelMotor.getDeviceID(),  MotorAlignmentValue.Opposed);
    followerMotor.setControl(follower);

    // Initialize and setup lookup tables
    lutRPM = new InterpolatingDoubleTreeMap();
    lutHood = new InterpolatingDoubleTreeMap();
    lookupTableSetup();
  }

  
  public void setFlywheelRPM(double rpm){
    rps = rpm / 60;
    flywheelMotor.setControl(flywheelVelocity.withVelocity(rps));
    
    SmartDashboard.putNumber("flywheel Speed", flywheelMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("follower Speed", followerMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("rpm", rpm);
  }
      
  public void runFeederMotor(){
    feederMotor.set(-0.8); //80% power
  }

  public void stopFlywheel(){
    flywheelMotor.stopMotor();
    feederMotor.stopMotor();
  }

  public void setHood(double angle){
    //ticks based
    
    //maxTicks = Constants.ShooterConstants.maxHoodTicks;


  }

  @Override
  public void periodic() {
    // double poseX = Aim.getInstance().diffToHub.getX();
    // double poseY = Aim.getInstance().diffToHub.getY();
    
    // double distanceToHub = Math.hypot(poseY,poseX);
    
    // double hoodAngle = hoodCalc(distanceToHub);

    // setHood(hoodAngle);
  }
  
  public double hoodCalc(double distance) {
    return lutHood.get(distance);
  }

  public double rpmCalc(double distance) {
    return lutRPM.get(distance);
  }

  private void lookupTableSetup(){
    //key is distance
    lutRPM.put(0.0,0.0);

    lutHood.put(0.0,0.0);
  }

  public static Shooter instance;
  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }
}
