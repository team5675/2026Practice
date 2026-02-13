// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCommand extends Command {
  public boolean isClimbFinished;
  /** Creates a new ClimberUp. */
  public ClimbCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    isClimbFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      Climber.getInstance().climberMotor.set(0.5);
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Climber.getInstance().climberMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(0 - Climber.getInstance().ticksEncoder.getPosition()) < 500;
  }
}
