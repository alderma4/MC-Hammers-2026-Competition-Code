package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.SpindexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public final class PreloadShoot {

  private PreloadShoot() {}

  // --- timings (tune) ---
  private static final double kSpinupTimeSec = 1.0;
  private static final double kFeedTimeSec   = 2.0;

  /** Uses the currently selected teleop preset (D-pad). */
  public static Command create(ShooterSubsystem shooter,
                               FeederSubsystem feeder,
                               SpindexerSubsystem spindexer) {
    return createWithRPM(shooter, feeder, spindexer, () -> shooter.getSelectedRPM());
  }

  /** Auto: force CLOSE distance RPM */
  public static Command close(ShooterSubsystem shooter,
                              FeederSubsystem feeder,
                              SpindexerSubsystem spindexer) {
    return createWithRPM(shooter, feeder, spindexer, () -> ShooterConstants.kCloseRPM);
  }

  /** Auto: force MEDIUM distance RPM */
  public static Command medium(ShooterSubsystem shooter,
                               FeederSubsystem feeder,
                               SpindexerSubsystem spindexer) {
    return createWithRPM(shooter, feeder, spindexer, () -> ShooterConstants.kMediumRPM);
  }

  /** Auto: force FAR distance RPM */
  public static Command far(ShooterSubsystem shooter,
                            FeederSubsystem feeder,
                            SpindexerSubsystem spindexer) {
    return createWithRPM(shooter, feeder, spindexer, () -> ShooterConstants.kFarRPM);
  }

  // --- internal helper ---
  private interface RpmSupplier { double get(); }

  private static Command createWithRPM(ShooterSubsystem shooter,
                                       FeederSubsystem feeder,
                                       SpindexerSubsystem spindexer,
                                       RpmSupplier rpmSupplier) {

    return Commands.sequence(
        // Spin up shooter
        Commands.run(() -> shooter.runAtRPM(rpmSupplier.get()), shooter)
                .withTimeout(kSpinupTimeSec),

        // Feed + spindex while keeping shooter running
        Commands.run(() -> {
                  shooter.runAtRPM(rpmSupplier.get());
                  feeder.runReverse();
                  spindexer.runAtRPM(SpindexerConstants.kPreloadFeedRPM);
                }, shooter, feeder, spindexer)
                .withTimeout(kFeedTimeSec)
    ).finallyDo(interrupted -> {
      feeder.stop();
      spindexer.stop();
      shooter.stopShooter();
    });
  }
}