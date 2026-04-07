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

  // Desired final distance from robot to HUB CENTER
  private static final double TARGET_DISTANCE_METERS = 0.9144;

  // Forward offset from tag plane to hub center.
  // Start at 0.0 and tune only if your tag is meaningfully forward/back from the actual hole center.
  private static final double TAG_TO_HUB_CENTER_FORWARD_METERS = 0.0;

  // Sideways offset from each tag to the hub center.
  // Tune this. Start around half the spacing between the two "center" tags on that hub face.
  private static final double TAG_TO_HUB_CENTER_RIGHT_METERS = 0.28;

  private static final double kP_TURN_FAR = 0.018;
  private static final double kP_TURN_NEAR = 0.024;

  private static final double kP_STRAFE_FAR = 0.35;
  private static final double kP_STRAFE_NEAR = 0.50;

  private static final double kP_FORWARD_FAR = 0.30;
  private static final double kP_FORWARD_NEAR = 0.40;

  private static final double MAX_TURN_FAR = 0.12;
  private static final double MAX_TURN_NEAR = 0.16;

  private static final double MAX_STRAFE_FAR = 0.12;
  private static final double MAX_STRAFE_NEAR = 0.16;

  private static final double MAX_FORWARD_FAR = 0.12;
  private static final double MAX_FORWARD_NEAR = 0.14;

  private static final double ANGLE_TOLERANCE_DEG = 2.0;
  private static final double STRAFE_TOLERANCE_M = 0.04;
  private static final double DIST_TOLERANCE_M = 0.05;

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

    double robotToTagForward = limelight.getForwardDistanceMeters();
    double robotToTagRight = limelight.getRightOffsetMeters();

    if (Double.isNaN(robotToTagForward) || Double.isNaN(robotToTagRight)) {
      stop();
      SmartDashboard.putString("LL Align Status", "Bad LL data");
      return;
    }

    // Estimate hub center relative to robot.
    // Tag 10 and 26 are on opposite sides of their face, so the sideways offset mirrors.
    double hubForward = robotToTagForward + TAG_TO_HUB_CENTER_FORWARD_METERS;
    double hubRight;

    if (activePreferredTag == TAG_10) {
      hubRight = robotToTagRight - TAG_TO_HUB_CENTER_RIGHT_METERS;
    } else {
      hubRight = robotToTagRight + TAG_TO_HUB_CENTER_RIGHT_METERS;
    }

    // Angle from robot forward direction to the hub center
    double desiredAngleDeg = Math.toDegrees(Math.atan2(hubRight, hubForward));
    double turnError = desiredAngleDeg;

    // Lateral error is now relative to HUB CENTER, not tag center
    double strafeError = hubRight;

    // Distance to HUB CENTER
    double hubDistance = Math.hypot(hubForward, hubRight);
    double distanceError = hubDistance - TARGET_DISTANCE_METERS;

    boolean nearMode = hubDistance < NEAR_DISTANCE_THRESHOLD_M;

    double kPTurn = nearMode ? kP_TURN_NEAR : kP_TURN_FAR;
    double kPStrafe = nearMode ? kP_STRAFE_NEAR : kP_STRAFE_FAR;
    double kPForward = nearMode ? kP_FORWARD_NEAR : kP_FORWARD_FAR;

    double maxTurn = nearMode ? MAX_TURN_NEAR : MAX_TURN_FAR;
    double maxStrafe = nearMode ? MAX_STRAFE_NEAR : MAX_STRAFE_FAR;
    double maxForward = nearMode ? MAX_FORWARD_NEAR : MAX_FORWARD_FAR;

    double turnCmd = -kPTurn * turnError;
    turnCmd = MathUtil.clamp(turnCmd, -maxTurn, maxTurn);

    if (Math.abs(turnError) < 6.0) {
      turnCmd *= 0.6;
    }
    if (Math.abs(turnError) < ANGLE_TOLERANCE_DEG) {
      turnCmd = 0.0;
    }

    double strafeCmd = -kPStrafe * strafeError;
    strafeCmd = MathUtil.clamp(strafeCmd, -maxStrafe, maxStrafe);

    if (Math.abs(strafeError) < 0.12) {
      strafeCmd *= 0.5;
    }
    if (Math.abs(strafeError) < STRAFE_TOLERANCE_M) {
      strafeCmd = 0.0;
    }

    double forwardCmd = kPForward * distanceError;
    forwardCmd = MathUtil.clamp(forwardCmd, -maxForward, maxForward);

    if (Math.abs(distanceError) < 0.20) {
      forwardCmd *= 0.45;
    }
    if (Math.abs(distanceError) < DIST_TOLERANCE_M) {
      forwardCmd = 0.0;
    }

    // Prevent pushing in hard while still aimed badly
    if (nearMode) {
      if (Math.abs(turnError) > 3.0 || Math.abs(strafeError) > 0.08) {
        forwardCmd = 0.0;
      }
    } else {
      if (Math.abs(turnError) > 6.0 || Math.abs(strafeError) > 0.16) {
        forwardCmd = 0.0;
      }
    }

    drive.robotCentricDrive(forwardCmd, strafeCmd, turnCmd);

    SmartDashboard.putString("LL Align Status", nearMode ? "Near Align" : "Far Align");
    SmartDashboard.putNumber("LL Seen Tag", seenTag);
    SmartDashboard.putNumber("LL Preferred Tag", activePreferredTag);

    SmartDashboard.putNumber("LL Robot->Tag Forward", robotToTagForward);
    SmartDashboard.putNumber("LL Robot->Tag Right", robotToTagRight);

    SmartDashboard.putNumber("LL Hub Forward", hubForward);
    SmartDashboard.putNumber("LL Hub Right", hubRight);
    SmartDashboard.putNumber("LL Desired Angle Deg", desiredAngleDeg);
    SmartDashboard.putNumber("LL Turn Error", turnError);
    SmartDashboard.putNumber("LL Turn Cmd", turnCmd);

    SmartDashboard.putNumber("LL Strafe Error", strafeError);
    SmartDashboard.putNumber("LL Strafe Cmd", strafeCmd);

    SmartDashboard.putNumber("LL Hub Distance", hubDistance);
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