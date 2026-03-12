package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax intakeMotor =
      new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);

  public IntakeSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
        .inverted(IntakeConstants.kInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.kCurrentLimit);

    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Pull game piece in */
  public void intakeIn() {
    intakeMotor.set(IntakeConstants.kInSpeed);
  }

  /** Spit game piece out */
  public void intakeOut() {
    intakeMotor.set(IntakeConstants.kOutSpeed);
  }

  public void stop() {
    intakeMotor.set(0.0);
  }
}