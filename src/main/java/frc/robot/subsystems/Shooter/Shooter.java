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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public TalonFX flywheelMotor;
  public TalonFX followerMotor;
  public SparkMax hoodMotor;
  public TalonFX providerMotor;
  public boolean isFlywheelActive = false;
  public boolean isProviderActive = false;
  public double distanceToHub;
  public Pose2d hubPose;
  public Follower follower;
  public double rps;
  private final VelocityTorqueCurrentFOC flywheelVelocity;
        
  public Shooter() {
    flywheelMotor = new TalonFX(Constants.ShooterConstants.flyWheelMotorId, "canivore");
    followerMotor = new TalonFX(Constants.ShooterConstants.followerMotorId, "canivore");
    hoodMotor = new SparkMax(Constants.ShooterConstants.hoodMotorId, MotorType.kBrushless);
    providerMotor = new TalonFX(Constants.ShooterConstants.providerMotorId, "canivore");

    flywheelVelocity =
    new VelocityTorqueCurrentFOC(0); 

    TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current limits — protect motor and battery
        config.CurrentLimits.StatorCurrentLimit = 80;
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
  }

  public void activateShooter() {
    //activate shooter at the start of the match
    if (isFlywheelActive == false) {
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

<<<<<<< HEAD
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
  
=======
  // blue hub (4.625, 4.0346, 0);
  // red hub (11.916, 4.0346, 0);
>>>>>>> dde57b35e4993be174ccdbe27c559f4601df8723

  public void startShooting() {
   if (isProviderActive == false) {
      providerMotor.set(1);
      isProviderActive = true;
   }
  }

  public void stopShooting() {
    providerMotor.set(0);
    isProviderActive = false;
  }

  public void setFlywheelRPM(double rpm){
  
  //  flywheelMotor.set(rpm);
  //   followerMotor.set(rpm);
  
        rps = rpm / 60;
        flywheelMotor.setControl(flywheelVelocity.withVelocity(rps));
        
        //followerMotor.setControl(flywheelVelocity.withVelocity(rps));

        SmartDashboard.putNumber("flywheel Speed", flywheelMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("follower Speed", followerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("rpm", rpm);
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
