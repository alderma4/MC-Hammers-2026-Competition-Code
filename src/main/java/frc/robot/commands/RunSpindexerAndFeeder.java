package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public class RunSpindexerAndFeeder extends Command {

  private final SpindexerSubsystem spindexerSubsystem;
  private final FeederSubsystem feederSubsystem;

  public RunSpindexerAndFeeder(SpindexerSubsystem spindexerSubsystem, FeederSubsystem feederSubsystem) {
    this.spindexerSubsystem = spindexerSubsystem;
    this.feederSubsystem = feederSubsystem;
    addRequirements(spindexerSubsystem, feederSubsystem);
  }

  @Override
  public void execute() {
    spindexerSubsystem.runAtRPM(SpindexerConstants.kFeedRPM);
    feederSubsystem.runForward();
  }

  @Override
  public void end(boolean interrupted) {
    spindexerSubsystem.stop();
    feederSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}