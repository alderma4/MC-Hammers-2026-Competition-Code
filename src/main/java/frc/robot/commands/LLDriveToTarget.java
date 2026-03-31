package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LLDriveToTarget extends Command {

  private final DriveSubsystem drive;
  private final LimelightSubsystem limelight;

  private static final int TAG_10 = 10;
  private static final int TAG_26 = 26;

  // -------------------------
  // GOALS
  // -------------------------
  private static final double TARGET_DISTANCE_METERS = 0.9144;

  // These are the shot-angle targets.
  // Start at 0.0. If the robot still points left/right of the hub center,
  // tune these separately.
  private static final double DESIRED_TX_TAG_10 = 0.0;
  private static final double DESIRED_TX_TAG_26 = 0.0;

  // This is for lining up the robot centerline with the hub opening.
  // Start at 0.0, then tune if needed.
  private static final double DESIRED_LATERAL_OFFSET_METERS = 0.0;

  // -------------------------
  // TUNING
  // -------------------------
  private static final double kP_TURN_FAR = 0.008;
  private static final double kP_TURN_NEAR = 0.012;

  private static final double kP_STRAFE_FAR = 0.25;
  private static final double kP_STRAFE_NEAR = 0.35;

  private static final double kP_FORWARD_FAR = 0.30;
  private static final double kP_FORWARD_NEAR = 0.45;

  private static final double MAX_TURN_FAR = 0.08;
  private static final double MAX_TURN_NEAR = 0.11;

  private static final double MAX_STRAFE_FAR = 0.07;
  private static final double MAX_STRAFE_NEAR = 0.10;

  private static final double MAX_FORWARD_FAR = 0.10;
  private static final double MAX_FORWARD_NEAR = 0.12;

  private static final double TX_TOLERANCE_DEG = 0.7;
  private static final double STRAFE_TOLERANCE_M = 0.06;
  private static final double DIST_TOLERANCE_M = 0.05;

  // Inside this distance, tighten the controls
  private static final double NEAR_DISTANCE_THRESHOLD_M = 1.30;

  private int activePreferredTag = TAG_10;

  public LLDriveToTarget(DriveSubsystem drive, LimelightSubsystem limelight) {
    this.drive = drive;
    this.limelight = limelight;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    activePreferredTag = TAG_10;
    limelight.setPriorityTag(activePreferredTag);
    SmartDashboard.putString("LL Align Status", "Started");
  }

  @Override
  public void execute() {
    int seenTag = limelight.getTagID();

    if (seenTag == TAG_10) {
      activePreferredTag = TAG_10;
      limelight.setPriorityTag(TAG_10);
    } else if (seenTag == TAG_26) {
      activePreferredTag = TAG_26;
      limelight.setPriorityTag(TAG_26);
    }

    if (!limelight.hasTarget()) {
      stop();
      SmartDashboard.putString("LL Align Status", "No target");
      return;
    }

    double tx = limelight.getTX();
    double distance = limelight.getForwardDistanceMeters();
    double lateral = limelight.getRightOffsetMeters();

    if (Double.isNaN(distance) || Double.isNaN(lateral)) {
      stop();
      SmartDashboard.putString("LL Align Status", "Bad LL data");
      return;
    }

    double desiredTx =
        (activePreferredTag == TAG_26) ? DESIRED_TX_TAG_26 : DESIRED_TX_TAG_10;

    double turnError = tx - desiredTx;
    double strafeError = lateral - DESIRED_LATERAL_OFFSET_METERS;
    double distanceError = distance - TARGET_DISTANCE_METERS;

    boolean nearMode = distance < NEAR_DISTANCE_THRESHOLD_M;

    double kPTurn = nearMode ? kP_TURN_NEAR : kP_TURN_FAR;
    double kPStrafe = nearMode ? kP_STRAFE_NEAR : kP_STRAFE_FAR;
    double kPForward = nearMode ? kP_FORWARD_NEAR : kP_FORWARD_FAR;

    double maxTurn = nearMode ? MAX_TURN_NEAR : MAX_TURN_FAR;
    double maxStrafe = nearMode ? MAX_STRAFE_NEAR : MAX_STRAFE_FAR;
    double maxForward = nearMode ? MAX_FORWARD_NEAR : MAX_FORWARD_FAR;

    // -------------------------
    // TURN
    // -------------------------
    double turnCmd = -kPTurn * turnError;
    turnCmd = MathUtil.clamp(turnCmd, -maxTurn, maxTurn);

    if (Math.abs(turnError) < 3.0) {
      turnCmd *= 0.5;
    }
    if (Math.abs(turnError) < TX_TOLERANCE_DEG) {
      turnCmd = 0.0;
    }

    // -------------------------
    // STRAFE
    // -------------------------
    double strafeCmd = -kPStrafe * strafeError;
    strafeCmd = MathUtil.clamp(strafeCmd, -maxStrafe, maxStrafe);

    if (Math.abs(strafeError) < 0.20) {
      strafeCmd *= 0.5;
    }
    if (Math.abs(strafeError) < STRAFE_TOLERANCE_M) {
      strafeCmd = 0.0;
    }

    // -------------------------
    // FORWARD
    // -------------------------
    double forwardCmd = kPForward * distanceError;
    forwardCmd = MathUtil.clamp(forwardCmd, -maxForward, maxForward);

    if (Math.abs(distanceError) < 0.25) {
      forwardCmd *= 0.4;
    }
    if (Math.abs(distanceError) < DIST_TOLERANCE_M) {
      forwardCmd = 0.0;
    }

    // Do not keep driving in if angle/centerline are still off.
    if (nearMode) {
      if (Math.abs(turnError) > 1.5 || Math.abs(strafeError) > 0.08) {
        forwardCmd = 0.0;
      }
    } else {
      if (Math.abs(turnError) > 2.5 || Math.abs(strafeError) > 0.15) {
        forwardCmd = 0.0;
      }
    }

    drive.robotCentricDrive(forwardCmd, strafeCmd, turnCmd);

    SmartDashboard.putString("LL Align Status", nearMode ? "Near Align" : "Far Align");
    SmartDashboard.putNumber("LL Seen Tag", seenTag);
    SmartDashboard.putNumber("LL Preferred Tag", activePreferredTag);

    SmartDashboard.putNumber("LL tx", tx);
    SmartDashboard.putNumber("LL Desired tx", desiredTx);
    SmartDashboard.putNumber("LL Turn Error", turnError);
    SmartDashboard.putNumber("LL Turn Cmd", turnCmd);

    SmartDashboard.putNumber("LL Lateral", lateral);
    SmartDashboard.putNumber("LL Strafe Error", strafeError);
    SmartDashboard.putNumber("LL Strafe Cmd", strafeCmd);

    SmartDashboard.putNumber("LL Distance M", distance);
    SmartDashboard.putNumber("LL Distance Error", distanceError);
    SmartDashboard.putNumber("LL Forward Cmd", forwardCmd);
  }

  @Override
  public void end(boolean interrupted) {
    stop();
    limelight.clearPriorityTag();
    SmartDashboard.putString("LL Align Status", interrupted ? "Interrupted" : "Ended");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void stop() {
    drive.robotCentricDrive(0.0, 0.0, 0.0);
  }
}