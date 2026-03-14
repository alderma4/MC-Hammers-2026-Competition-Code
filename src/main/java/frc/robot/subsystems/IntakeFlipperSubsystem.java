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
import frc.robot.Constants.IntakeFlipperConstants;

public class IntakeFlipperSubsystem extends SubsystemBase {

  private final SparkMax flipperMotor =
      new SparkMax(IntakeFlipperConstants.kFlipperCanId, MotorType.kBrushless);

  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  public IntakeFlipperSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
        .inverted(IntakeFlipperConstants.kInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeFlipperConstants.kCurrentLimit);

    // Convert motor rotations -> flipper OUTPUT rotations
    config.encoder.positionConversionFactor(1.0 / IntakeFlipperConstants.kTotalGearRatio);
    config.encoder.velocityConversionFactor(
        (1.0 / IntakeFlipperConstants.kTotalGearRatio) * (1.0 / 60.0));

    // Enable position closed-loop on the built-in NEO encoder
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(IntakeFlipperConstants.kP, IntakeFlipperConstants.kI, IntakeFlipperConstants.kD)
        .outputRange(-1.0, 1.0);

    flipperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = flipperMotor.getEncoder();
    pid = flipperMotor.getClosedLoopController();

    // DO NOT command a position here.
    // Relative encoders do not know true mechanism position on startup.
  }

  /** Zero encoder when flipper is physically fully IN */
  public void zeroAtInPosition() {
    encoder.setPosition(0.0);
  }

  /** Move to OUT (deployed) setpoint */
  public void moveToOutPosition() {
    pid.setReference(IntakeFlipperConstants.kOutPosition, ControlType.kPosition);
  }

  /** Move to IN (stowed) setpoint */
  public void moveToInPosition() {
    pid.setReference(IntakeFlipperConstants.kInPosition, ControlType.kPosition);
  }

  /** Manual OUT (deploy) while held */
  public void manualFlipOut() {
    flipperMotor.set(IntakeFlipperConstants.kFlipOutSpeed);
  }

  /** Manual IN (stow) while held */
  public void manualFlipIn() {
    flipperMotor.set(IntakeFlipperConstants.kFlipInSpeed);
  }

  /** Stop manual movement */
  public void stopManual() {
    flipperMotor.set(0.0);
  }

  // Compatibility methods
  public void flipIn() {
    moveToInPosition();
  }

  public void flipOut() {
    moveToOutPosition();
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getPositionDegrees() {
    return encoder.getPosition() * 360.0;
  }

  public boolean isNearInPosition() {
    return Math.abs(getPosition() - IntakeFlipperConstants.kInPosition) < 0.03;
  }

  public boolean isNearOutPosition() {
    return Math.abs(getPosition() - IntakeFlipperConstants.kOutPosition) < 0.03;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Flipper Pos (output rotations)", encoder.getPosition());
    SmartDashboard.putNumber("Intake Flipper Vel (output rps)", encoder.getVelocity());
    SmartDashboard.putBoolean("Intake Flipper Near In", isNearInPosition());
    SmartDashboard.putBoolean("Intake Flipper Near Out", isNearOutPosition());
  }
}