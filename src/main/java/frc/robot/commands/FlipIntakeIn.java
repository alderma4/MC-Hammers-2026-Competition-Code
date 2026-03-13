package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeFlipperSubsystem;

public class FlipIntakeIn extends Command {

  private final IntakeFlipperSubsystem flipper;

  public FlipIntakeIn(IntakeFlipperSubsystem flipper) {
    this.flipper = flipper;
    addRequirements(flipper);
  }

  @Override
  public void initialize() {
    flipper.moveToInPosition();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}