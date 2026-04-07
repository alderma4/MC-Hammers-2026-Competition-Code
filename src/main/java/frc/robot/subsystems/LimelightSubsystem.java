package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

  private static final String LIMELIGHT_NAME = "limelight";

  private final NetworkTable table =
      NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);

  public LimelightSubsystem() {}

  public boolean hasTarget() {
    return table.getEntry("tv").getDouble(0.0) == 1.0;
  }

  public double getTX() {
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getTY() {
    return table.getEntry("ty").getDouble(0.0);
  }

  public double getTA() {
    return table.getEntry("ta").getDouble(0.0);
  }

  public int getTagID() {
    return (int) table.getEntry("tid").getDouble(-1.0);
  }

  public double[] getRawFiducials() {
    return table.getEntry("rawfiducials").getDoubleArray(new double[0]);
  }

  public double[] getTargetPoseRobotSpace() {
    return table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
  }

  /**
   * Returns distance from ROBOT to the primary target.
   * Limelight rawfiducials format:
   * [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity, ...]
   */
  public double getForwardDistanceMeters() {
    if (!hasTarget()) {
      return Double.NaN;
    }

    double[] raw = getRawFiducials();
    if (raw.length < 6) {
      return Double.NaN;
    }

    return raw[5]; // distToRobot
  }

  public double getForwardDistanceFeet() {
    double meters = getForwardDistanceMeters();
    if (Double.isNaN(meters)) {
      return Double.NaN;
    }
    return meters * 3.28084;
  }

  /**
   * Horizontal offset from robot center to target in robot space.
   * Keeping pose[0] because that matches your current testing setup.
   */
  public double getRightOffsetMeters() {
    if (!hasTarget()) {
      return Double.NaN;
    }

    double[] pose = getTargetPoseRobotSpace();
    if (pose.length < 1) {
      return Double.NaN;
    }

    return pose[0];
  }

  public void setPriorityTag(int tagId) {
    table.getEntry("priorityid").setNumber(tagId);
  }

  public void clearPriorityTag() {
    table.getEntry("priorityid").setNumber(-1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("LL Has Target", hasTarget());
    SmartDashboard.putNumber("LL tx", getTX());
    SmartDashboard.putNumber("LL ty", getTY());
    SmartDashboard.putNumber("LL ta", getTA());
    SmartDashboard.putNumber("LL Tag ID", getTagID());

    SmartDashboard.putNumber("LL Forward Dist M", getForwardDistanceMeters());
    SmartDashboard.putNumber("LL Forward Dist Ft", getForwardDistanceFeet());
    SmartDashboard.putNumber("LL Right Offset M", getRightOffsetMeters());

    double[] raw = getRawFiducials();
    SmartDashboard.putNumber("LL Raw Fiducials Length", raw.length);
  }
}