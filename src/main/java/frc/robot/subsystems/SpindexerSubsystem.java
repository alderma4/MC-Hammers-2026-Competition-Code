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

import frc.robot.Constants.SpindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {

  private final SparkMax motor =
      new SparkMax(SpindexerConstants.kSpindexerCanId, MotorType.kBrushless);

  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  public SpindexerSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
        .inverted(SpindexerConstants.kInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(SpindexerConstants.kCurrentLimit);

    // Keep velocity in RPM
    config.encoder.velocityConversionFactor(1.0);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(SpindexerConstants.kP, SpindexerConstants.kI, SpindexerConstants.kD)
        .velocityFF(SpindexerConstants.kFF)
        .outputRange(-1.0, 1.0);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Spindexer RPM", encoder.getVelocity());
  }

  /** Run spindexer at a specific RPM (closed-loop velocity control) */
  public void runAtRPM(double rpm) {
    pid.setReference(rpm, ControlType.kVelocity);
  }

  /** Normal forward feed RPM (teleop) */
  public void runFeedRPM() {
    runAtRPM(SpindexerConstants.kFeedRPM);
  }

  /** Reverse feed RPM (used for clearing jams) */
  public void runReverseRPM() {
    runAtRPM(-SpindexerConstants.kFeedRPM);
  }

  /** Stop motor */
  public void stop() {
    motor.set(0.0);
  }
}