package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {

  private final SparkMax feederMotor =
      new SparkMax(FeederConstants.kFeederCanId, MotorType.kBrushless);

  public FeederSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
        .inverted(FeederConstants.kFeederInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(FeederConstants.kCurrentLimit);

    feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runForward() {
    feederMotor.set(FeederConstants.kForwardSpeed);
  }

  public void runReverse() {
    feederMotor.set(FeederConstants.kReverseSpeed);
  }

  public void stop() {
    feederMotor.set(0.0);
  }
}