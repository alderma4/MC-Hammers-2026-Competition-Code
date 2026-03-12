package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LLDriveToTarget extends Command {

  private final DriveSubsystem drive;
  private final LimelightSubsystem limelight;

  // TURN: tx -> 0
  // Slight D helps keep it stable at higher speeds
  private final PIDController turnPid = new PIDController(0.016, 0.0, 0.001);

  // DRIVE: ta -> setpoint
  // Higher P = more aggressive = faster approach
  private final PIDController drivePid = new PIDController(1.1, 0.0, 0.0);

  // Bigger = stop closer, smaller = stop farther
  private static final double kTargetAreaSetpoint = 4.0;

  // Ignore tiny tx noise
  private static final double kTxDeadbandDeg = 1.0;

  // If target disappears briefly, keep driving smoothly
  private static final double kTargetGraceSec = 0.12;

  // ✅ FASTER caps
  private static final double kMaxDrive = 0.90;
  private static final double kMaxTurn  = 0.85;

  // ✅ Snappier smoothing (higher = faster response)
  private static final double kAlpha = 0.30;

  // --- State ---
  private double lastTx = 0.0;
  private double lastTa = 0.0;
  private double lastSeenTime = -999.0;

  private double lastRotCmd = 0.0;
  private double lastFwdCmd = 0.0;

  public LLDriveToTarget(DriveSubsystem drive, LimelightSubsystem limelight) {
    this.drive = drive;
    this.limelight = limelight;

    turnPid.setTolerance(1.0);
    drivePid.setTolerance(0.25);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    turnPid.reset();
    drivePid.reset();

    lastRotCmd = 0.0;
    lastFwdCmd = 0.0;

    lastSeenTime = -999.0;
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();

    if (limelight.hasTarget()) {
      lastTx = limelight.getTX();
      lastTa = limelight.getTA();
      lastSeenTime = now;
    }

    boolean recentlySeen = (now - lastSeenTime) <= kTargetGraceSec;
    if (!recentlySeen) {
      drive.robotCentricDrive(0.0, 0.0, 0.0);
      lastRotCmd = 0.0;
      lastFwdCmd = 0.0;
      return;
    }

    double tx = lastTx;
    double ta = lastTa;

    if (Math.abs(tx) < kTxDeadbandDeg) tx = 0.0;

    // Turn command
    double rot = turnPid.calculate(tx, 0.0);
    rot = clamp(rot, -kMaxTurn, kMaxTurn);
    if (turnPid.atSetpoint()) rot = 0.0;

    // Forward command
    double forward = drivePid.calculate(ta, kTargetAreaSetpoint);
    forward = clamp(forward, -kMaxDrive, kMaxDrive);
    if (drivePid.atSetpoint()) forward = 0.0;

    // Smooth (still smooth, but faster ramp)
    rot = lowPass(lastRotCmd, rot, kAlpha);
    forward = lowPass(lastFwdCmd, forward, kAlpha);

    lastRotCmd = rot;
    lastFwdCmd = forward;

    drive.robotCentricDrive(forward, 0.0, rot);
  }

  @Override
  public void end(boolean interrupted) {
    drive.robotCentricDrive(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }

  private static double lowPass(double prev, double target, double alpha) {
    return prev + alpha * (target - prev);
  }
}