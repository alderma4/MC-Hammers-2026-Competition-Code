package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  // ✅ 3-distance preset selector
  public enum ShooterMode {
    CLOSE,
    MEDIUM,
    FAR
  }

  private ShooterMode mode = ShooterMode.MEDIUM; // default

  private final SparkMax shooterLeft =
      new SparkMax(ShooterConstants.kShooterLeftCanId, MotorType.kBrushless);
  private final SparkMax shooterRight =
      new SparkMax(ShooterConstants.kShooterRightCanId, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final SparkClosedLoopController leftPID;
  private final SparkClosedLoopController rightPID;

  public ShooterSubsystem() {
    // Left config
    SparkMaxConfig leftCfg = new SparkMaxConfig();
    leftCfg
        .inverted(ShooterConstants.kShooterLeftInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.kCurrentLimit);

    leftCfg.encoder.velocityConversionFactor(1.0); // keep RPM

    leftCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD)
        .velocityFF(ShooterConstants.kFF)
        .outputRange(-1.0, 1.0);

    shooterLeft.configure(leftCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Right config
    SparkMaxConfig rightCfg = new SparkMaxConfig();
    rightCfg
        .inverted(ShooterConstants.kShooterRightInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.kCurrentLimit);

    rightCfg.encoder.velocityConversionFactor(1.0); // keep RPM

    rightCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD)
        .velocityFF(ShooterConstants.kFF)
        .outputRange(-1.0, 1.0);

    shooterRight.configure(rightCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftEncoder = shooterLeft.getEncoder();
    rightEncoder = shooterRight.getEncoder();

    leftPID = shooterLeft.getClosedLoopController();
    rightPID = shooterRight.getClosedLoopController();

    SmartDashboard.putString("Shooter Mode", mode.name());
    SmartDashboard.putNumber("Shooter Target RPM", getSelectedRPM());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Left RPM", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Right RPM", rightEncoder.getVelocity());
    SmartDashboard.putString("Shooter Mode", mode.name());
    SmartDashboard.putNumber("Shooter Target RPM", getSelectedRPM());
  }

  public void setMode(ShooterMode newMode) {
    mode = newMode;
    SmartDashboard.putString("Shooter Mode", mode.name());
    SmartDashboard.putNumber("Shooter Target RPM", getSelectedRPM());
  }

  public ShooterMode getMode() {
    return mode;
  }

  public double getSelectedRPM() {
    switch (mode) {
      case CLOSE:
        return ShooterConstants.kCloseRPM;
      case FAR:
        return ShooterConstants.kFarRPM;
      case MEDIUM:
      default:
        return ShooterConstants.kMediumRPM;
    }
  }

  public void runAtRPM(double rpm) {
    leftPID.setReference(rpm, ControlType.kVelocity);
    rightPID.setReference(rpm, ControlType.kVelocity);
  }

  // ✅ Run whatever preset is currently selected
  public void runSelectedRPM() {
    runAtRPM(getSelectedRPM());
  }

  // Kept for compatibility with older code
  public void runAtTargetRPM() {
    runSelectedRPM();
  }

  public void stopShooter() {
    shooterLeft.set(0.0);
    shooterRight.set(0.0);
  }
}