package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;

public class RunShooterLow extends Command {

  private final ShooterSubsystem shooterSubsystem;

  public RunShooterLow(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setMode(ShooterMode.CLOSE);
  }

  @Override
  public void execute() {
    shooterSubsystem.runSelectedRPM();
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}