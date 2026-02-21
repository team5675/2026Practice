package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Aim;

public class ShootCommand extends Command {
  private double rpm;
  public double RPM_NEEDED;
  public double setRPM;
  
  Shooter shooter;
  public ShootCommand() {
    shooter = Shooter.getInstance();
  }

  @Override
  public void initialize() {
    rpm = RPM_NEEDED;
  }

  @Override
  public void execute() {

    RPM_NEEDED = shooter.rpmCalc(Aim.getInstance().distanceToHub);

    if(shooter.feederMotor.getStatorCurrent().getValueAsDouble() >= 20.0){
      rpm = RPM_NEEDED + 400;
    }else{
      rpm = RPM_NEEDED;
    }

    shooter.setFlywheelRPM(rpm);

  }

  @Override
  public void end(boolean interrupted) {
    shooter.setFlywheelRPM(1000); //idle speed
    shooter.feederMotor.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
