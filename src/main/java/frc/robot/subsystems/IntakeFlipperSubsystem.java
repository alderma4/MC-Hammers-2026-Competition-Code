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
    config.encoder.velocityConversionFactor((1.0 / IntakeFlipperConstants.kTotalGearRatio) * (1.0 / 60.0));

    // Enable position closed-loop on the built-in NEO encoder
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(IntakeFlipperConstants.kP, IntakeFlipperConstants.kI, IntakeFlipperConstants.kD)
        .outputRange(-1.0, 1.0);

    flipperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = flipperMotor.getEncoder();
    pid = flipperMotor.getClosedLoopController();

    // Start robot with flipper commanded to IN position
    moveToInPosition();
  }

  // ---------------- SET POSITION CONTROL ----------------

  /** Move to OUT (deployed) setpoint */
  public void moveToOutPosition() {
    pid.setReference(IntakeFlipperConstants.kOutPosition, ControlType.kPosition);
  }

  /** Move to IN (stowed) setpoint */
  public void moveToInPosition() {
    pid.setReference(IntakeFlipperConstants.kInPosition, ControlType.kPosition);
  }

  // ---------------- MANUAL OVERRIDE (OPEN LOOP) ----------------
  // These are for Start/Back manual control.

  /** Manual OUT (deploy) while held */
  public void manualFlipOut() {
    flipperMotor.set(IntakeFlipperConstants.kFlipOutSpeed);
  }

  /** Manual IN (stow) while held */
  public void manualFlipIn() {
    flipperMotor.set(IntakeFlipperConstants.kFlipInSpeed);
  }

  /** Stop manual movement (motor output 0) */
  public void stopManual() {
    flipperMotor.set(0.0);
  }

  // Compatibility (if anything still calls these)
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Flipper Pos (output rotations)", encoder.getPosition());
    SmartDashboard.putNumber("Intake Flipper Vel (output rps)", encoder.getVelocity());
  }
}