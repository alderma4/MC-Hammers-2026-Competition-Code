package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeFlipperSubsystem;

public class FlipIntakeOut extends Command {

  private final IntakeFlipperSubsystem flipper;

  public FlipIntakeOut(IntakeFlipperSubsystem flipper) {
    this.flipper = flipper;
    addRequirements(flipper);
  }

  @Override
  public void initialize() {
    flipper.moveToOutPosition();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}