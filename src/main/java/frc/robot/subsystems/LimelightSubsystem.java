package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HammerToolbox.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {

  // If your limelight name is NOT "limelight", change this.
  private static final String kLimelightName = "limelight";

  public boolean hasTarget() {
    return LimelightHelpers.getTV(kLimelightName);
  }

  public double getTX() {
    return LimelightHelpers.getTX(kLimelightName);
  }

  public double getTA() {
    return LimelightHelpers.getTA(kLimelightName);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("LL Has Target", hasTarget());
    SmartDashboard.putNumber("LL tx", getTX());
    SmartDashboard.putNumber("LL ta", getTA());
  }
}