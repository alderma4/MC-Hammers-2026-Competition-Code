package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HammerToolbox.HammerParts.PIDGains;

public final class Constants {

    public static class ModuleConstants {

        public static final int kTurnMotorCurrentLimit = 25;
        public static final int kDriveMotorCurrentLimit = 30;
        public static final int kElevatorMotorCurrentLimit = 25;
        public static final int kIntakeMotorCurrentLimit = 20;

        public static final double kdriveGearRatio = 1d / 6.75;
        public static final double kturnGearRatio = 1d / (150d / 7d);

        public static final double kwheelCircumference = Units.inchesToMeters(4) * Math.PI;

        public static final double kflipperIn = .5;
        public static final double kflipperOut = 0;

        public static final double kMaxModuleSpeedMetersPerSecond = Units.feetToMeters(14.5);

        public static final double ksVolts = .1;
        public static final double kDriveFeedForward = .2;
        public static final double kElevatorFeedForward = .001;

        public static final double kvTurning = .43205;
        public static final double ksTurning = .17161;

        public static final int kFrontLeftDriveMotorPort = 11;
        public static final int kRearLeftDriveMotorPort = 12;
        public static final int kRearRightDriveMotorPort = 13;
        public static final int kFrontRightDriveMotorPort = 14;

        public static final int kFrontLeftTurningMotorPort = 21;
        public static final int kRearLeftTurningMotorPort = 22;
        public static final int kRearRightTurningMotorPort = 23;
        public static final int kFrontRightTurningMotorPort = 24;

        public static final int kFrontLeftTurningEncoderPort = 1;
        public static final int kRearLeftTurningEncoderPort = 2;
        public static final int kRearRightTurningEncoderPort = 3;
        public static final int kFrontRightTurningEncoderPort = 4;

        public static final double kFrontLeftAngleZero = .719727;
        public static final double kRearLeftAngleZero = .277344;
        public static final double kRearRightAngleZero = .641846;
        public static final double kFrontRightAngleZero = .034180;

        public static final PIDGains kModuleDriveGains = new PIDGains(.1, 0, 0);
        public static final PIDGains kModuleTurningGains = new PIDGains(1.5, 0, 0.0016);

        public static int kFlipperMotorCurrentLimit = 40;
        public static double kflipMotorGearRatio = 16;
        public static double kflipperCircumference = 1;
        public static double kflipperFeedForward = .01;

        public static int kClimbMotorCurrentLimit = 20;
        public static double kclimbGearRatio = 100;
        public static double kclimbCircumference = 1.57;
        public static double kclimbFeedForward = .2;
    }

    public static class DriveConstants {

        public static final double kMaxSneakMetersPerSecond = 1.0;
        public static final double kMaxSpeedMetersPerSecond = 4.5;
        public static final double kMaxTurboMetersPerSecond = 8.0;

        public static final double kMaxRPM = 10;

        public static final int kPigeonPort = 20;

        public static final double kBumperToBumperWidth = Units.inchesToMeters(36.0);

        public static final double kTrackWidth = Units.inchesToMeters(21.25);
        public static final double kWheelBase = Units.inchesToMeters(21.25);

        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final boolean kGyroReversed = false;
        public static final boolean kFeildCentric = true;
        public static final boolean kGyroTurning = true;

        public static final PIDGains kGyroTurningGains = new PIDGains(.025, 0, 0);
        public static final double kMaxTurningVelocityDegrees = 20;
        public static final double kMaxTurningAcceleratonDegrees = 10;
        public static final double kGyroTurnTolerance = 2;
    }

    public static class AutoConstants {

        public static class PathPLannerConstants {

            public static final PIDGains kPPDriveGains = new PIDGains(1.0, 0, 0);
            public static final PIDGains kPPTurnGains = new PIDGains(1.5, 0, 0);

            public static final double kPPMaxVelocity = 4.5;
            public static final double kPPMaxAcceleration = 3.0;

            public static final HashMap<String, Command> kPPEventMap =
                new HashMap<String, Command>() {
                    {
                        // put("TargetTape", new LLAlignCommand(false));
                        // put("TargetTag", new LLAlignCommand(true));
                        // PathPlanner preload shot events should be added in RobotContainer,
                        // where subsystem instances exist.
                    }
                };
        }

        public static final double kScoreSequenceDropTime = 3;

        public static final PIDGains kTurnCommandGains = new PIDGains(.004, 0, 0);
        public static final double kTurnCommandMaxVelocity = 1;
        public static final double kTurnCommandMaxAcceleration = 1;
        public static final double kTurnCommandToleranceDeg = 0.5;
        public static final double kTurnCommandRateToleranceDegPerS = 0;

        public static final double kBalnaceCommandDeadbandDeg = -2.5;
        public static final PIDGains kBalanceCommandGains = new PIDGains(.019, 0, 0);
        public static final double kMaxBalancingVelocity = 1000;
        public static final double kMaxBalancingAcceleration = 5000;
    }

    public static class OperatorConstants {
        public static final int kDriveJoystickPort = 0;
        public static final int kTurnJoystickPort = 1;
        public static final int kOperatorControllerPort = 2;

        public static final double KDeadBand = .125;
        public static final double kJoystickPow = 2.5;
    }

    public static class LimelightConstants {

        // Limelight table name (NetworkTables)
        public static final String kLimelightName = "limelight";

        // pipelines
        public static final int kCubePipeline = 0;
        public static final int kReflectivePipeline = 1;
        public static final int kApriltagPipeline = 2;

        // PID values for limelight
        public static final PIDGains kLLTargetGains = new PIDGains(0.008, 0, 0);

        public static final PIDGains kLLPuppyTurnGains = new PIDGains(0.02, 0, 0);
        public static final PIDGains kLLPuppyDriveGains = new PIDGains(0.008, 0, 0);
        public static final double kPuppyTurnMotionSmoothing = 0.3;
        public static final double kPuppyDriveMotionSmoothing = 0.4;

        public static final PIDGains kLLAlignStrafeGains = new PIDGains(.04, 0.0015, 0.001);
        public static final PIDGains kLLAlignDriveGains = new PIDGains(.025, 0.0015, 0.0005);
        public static final double kAlignDriveMotionSmoothing = 0;
        public static final double kAlignStrafeMotionSmoothing = 0;
    }

    public static final String kRioCANBusName = "rio";

    public static final int XBOXCONTROLLER_ID = 2;

    public static final int BUTTON_12_ID = 12;
    public static final int BUTTON_11_ID = 11;
    public static final int BUTTON_10_ID = 10;
    public static final int BUTTON_9_ID = 9;
    public static final int BUTTON_8_ID = 8;
    public static final int BUTTON_7_ID = 7;
    public static final int BUTTON_2_ID = 2;
    public static final int BUTTON_1_ID = 1;

    public static final double MAX_INT_ENCODER = 0.5;
    public static final double MIN_INT_ENCODER = 0.0;

    /* ================= SHOOTER ================= */
    public static final class ShooterConstants {

        public static final int kShooterLeftCanId = 6;
        public static final int kShooterRightCanId = 9;

        public static final boolean kShooterLeftInverted = true;
        public static final boolean kShooterRightInverted = false;

        public static final int kCurrentLimit = 40;

        public static final double kCloseRPM = 2500.0;
        public static final double kMediumRPM = 3100.0;
        public static final double kFarRPM = 3250.0;

        // Auto-specific preload RPMs
        public static final double kCenterAutoRPM = 2800.0;
        public static final double kLeftAutoRPM   = 3300.0;
        public static final double kRightAutoRPM  = 3400.0;

        public static final double kP = 0.0003;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.00018;
    }

    /* ================= FEEDER ================= */
    public static final class FeederConstants {

        public static final int kFeederCanId = 7;

        public static final boolean kFeederInverted = false;
        public static final int kCurrentLimit = 30;

        public static final double kForwardSpeed = .5; // alderman changed this 3/24/2026
        public static final double kReverseSpeed = -.5;
    }

    /* ================= SPINDEXER ================= */
    public static final class SpindexerConstants {

        public static final int kSpindexerCanId = 17;

        public static final boolean kInverted = false;
        public static final int kCurrentLimit = 30;

        public static final double kP = 0.0002;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.00018;

        public static final double kFeedRPM = 7000.0;
        public static final double kPreloadFeedRPM = 7000.0;
    }

    /* ================= INTAKE ================= */
    public static final class IntakeConstants {

        public static final int kIntakeCanId = 8;

        public static final boolean kInverted = false;
        public static final int kCurrentLimit = 40;

        public static final double kGearRatio = 4.0;

        public static final double kInSpeed = 1;
        public static final double kOutSpeed = -1;
    }

    /* ================= INTAKE FLIPPER ================= */
    public static final class IntakeFlipperConstants {

        public static final int kFlipperCanId = 5;

        public static final boolean kInverted = false;
        public static final int kCurrentLimit = 40;

        public static final double kStage1GearRatio = 5.0;
        public static final double kStage2GearRatio = 5.0;
        public static final double kTotalGearRatio = kStage1GearRatio * kStage2GearRatio;

        // IMPORTANT:
        // Encoder is zeroed when the flipper is physically in.
        public static final double kInPosition = 0;

        // Tune this after zeroing if needed.
        // Start with your old distance between in and out.
        public static final double kOutPosition = -1.380942225456238;

        public static final double kP = 1.250;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kFlipInSpeed = 0.5;
        public static final double kFlipOutSpeed = -0.7;
    }

    private Constants() {}
}