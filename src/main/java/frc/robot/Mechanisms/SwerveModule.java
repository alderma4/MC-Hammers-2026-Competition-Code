// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms;


import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.BaseStatusSignal;


import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

//import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
//import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.HammerToolbox.HammerParts.PIDGains;

public class SwerveModule {
	/** Creates a new SwerveModule. */

	private final SparkMax driveMotor;
	private final SparkMax turnMotor;

	private final CoreCANcoder absoluteEncoder;

	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder turnEncoder;

	private final SparkClosedLoopController drivePID;
	private final SparkClosedLoopController turnPID;
	//private final ProfiledPIDController m_turningPIDController;

	public final double angleZero;
	private final String moduleName;

	SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
			ModuleConstants.ksTurning, ModuleConstants.kvTurning);
	
	public SwerveModule(
			String moduleName,
			int driveMotorChannel,
			int turningMotorChannel,
			int absoluteEncoderPort,
			double angleZero,
			PIDGains angularPIDGains,
			PIDGains drivePIDGains) {

		this.moduleName = moduleName;
		this.angleZero = angleZero;

		// Initialize the motors and encoders

		this.driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
		SparkMaxConfig configD = new SparkMaxConfig();
		configD
				.inverted(false)
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit);
		configD.encoder
				.positionConversionFactor(ModuleConstants.kdriveGearRatio * ModuleConstants.kwheelCircumference)
				.velocityConversionFactor(ModuleConstants.kdriveGearRatio * ModuleConstants.kwheelCircumference	* (1d / 60d));
		configD.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.velocityFF(ModuleConstants.kDriveFeedForward)
				.outputRange(-1, 1)
				.pid(drivePIDGains.kP,drivePIDGains.kI,drivePIDGains.kD);
		driveMotor.configure(configD, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		this.turnMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);
		SparkMaxConfig configT = new SparkMaxConfig();
		configT
				.inverted(true)
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(ModuleConstants.kTurnMotorCurrentLimit);
		configT.encoder
				.positionConversionFactor((2 * Math.PI) * ModuleConstants.kturnGearRatio)
				.velocityConversionFactor((2 * Math.PI) * ModuleConstants.kturnGearRatio * (1d / 60d));
		configT.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pid(angularPIDGains.kP,angularPIDGains.kI,angularPIDGains.kD)
				.positionWrappingEnabled(true)
				.positionWrappingMinInput(-Math.PI)
				.positionWrappingMaxInput(Math.PI);
		turnMotor.configure(configT, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		this.driveEncoder = driveMotor.getEncoder();
		this.turnEncoder = turnMotor.getEncoder();

		this.drivePID = driveMotor.getClosedLoopController();
	    this.turnPID = turnMotor.getClosedLoopController();

		// Initalize CANcoder
		absoluteEncoder = new CoreCANcoder(absoluteEncoderPort);
		Timer.delay(1);
		CANcoderConfiguration configs = new CANcoderConfiguration();
			BaseStatusSignal.setUpdateFrequencyForAll(100, absoluteEncoder.getPosition(), absoluteEncoder.getFault_Undervoltage(), absoluteEncoder.getSupplyVoltage());
				configs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
				configs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		absoluteEncoder.getConfigurator().apply(configs);		
		absoluteEncoder.clearStickyFaults();

		// var tEpositionRotation = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
		// var tEposition = tEpositionRotation * 360;
	    // turnEncoder.setPosition(tEposition - angleZero);
		var tEpositionRotation = absoluteEncoder.getAbsolutePosition().getValueAsDouble(); // Rotations (0 to 1)
   		var tEpositionRadians = tEpositionRotation * (2 * Math.PI); // Convert to radians
    	var angleZeroRadians = angleZero * (2 * Math.PI); // Convert angleZero from degrees to radians
    	turnEncoder.setPosition(tEpositionRadians - angleZeroRadians);

		SmartDashboard.putNumber(this.moduleName + " Offset", angleZero);
		SmartDashboard.putString(this.moduleName + " Abs. Status", absoluteEncoder.getFaultField().toString());
	}

	// Returns headings of the module
	public double getAbsoluteHeading() {
		return turnEncoder.getPosition();
	}

	public double getDistanceMeters() {
		return driveEncoder.getPosition();
	}

	// Returns current position of the modules
	public SwerveModulePosition getPosition() {
		double moduleAngleRadians = turnEncoder.getPosition();
		double distanceMeters = driveEncoder.getPosition();
		return new SwerveModulePosition(distanceMeters, new Rotation2d(moduleAngleRadians));
	}

	public void setDesiredState(SwerveModuleState desiredState) {		
			double moduleAngleRadians = turnEncoder.getPosition();
		// Optimize the reference state to avoid spinning further than 90 degrees to
		// desired state
		desiredState.optimize(new Rotation2d(moduleAngleRadians));

		drivePID.setReference(
				desiredState.speedMetersPerSecond,
				ControlType.kVelocity);

		turnPID.setReference(
				desiredState.angle.getRadians(),
				ControlType.kPosition);

		SmartDashboard.putNumber(this.moduleName + " Optimized Angle", desiredState.angle.getDegrees());
		SmartDashboard.putNumber(this.moduleName + " Turn Motor Output", turnMotor.getAppliedOutput());
		SmartDashboard.putNumber(this.moduleName + " Drive Motor Output", driveMotor.getAppliedOutput());
		
	}

	public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(), 
            new Rotation2d(turnEncoder.getPosition())
        );
    }


	public void resetEncoders() {
		driveEncoder.setPosition(0);
		turnEncoder.setPosition(0);
		var tEpositionRotation = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
		var tEposition = tEpositionRotation * (2 * Math.PI);
	    turnEncoder.setPosition(tEposition - (angleZero * (2 * Math.PI)));
	}

	public void stopMotors() {
		driveMotor.stopMotor();
		turnMotor.stopMotor();
	}

}
