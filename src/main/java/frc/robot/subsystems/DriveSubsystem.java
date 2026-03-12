// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// DISCLAIMER! THIS DRIVE_SUBSYSTEM HAS MANY BUGS AND SHOULD NOT BE USED AS A REFERENCE!


package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.*;
import com.studica.frc.AHRS.NavXComType;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Mechanisms.SwerveModule;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.HammerToolbox.LimelightHelpers;


public class DriveSubsystem extends SubsystemBase {


    private boolean fieldRelative = true;
    private boolean gyroTurning = false;
    private double targetRotationDegrees;


    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule rearLeft;
    private final SwerveModule rearRight;


    private SwerveModulePosition[] swervePosition;


    // Initalizing the gyro sensor
    private final AHRS gyro;


    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometry;


    // PID controller for gyro turning
    private ProfiledPIDController gyroTurnPidController;


    private final Field2d field = new Field2d();


    private SwerveDrivePoseEstimator poseEstimator;


    // Vision filtering
    private static final double kMaxVisionDistanceJumpMeters = 3.0;
    private static final double kMinTargetArea = 0.10;


    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {


        frontLeft = new SwerveModule(
                "FL",
                ModuleConstants.kFrontLeftDriveMotorPort,
                ModuleConstants.kFrontLeftTurningMotorPort,
                ModuleConstants.kFrontLeftTurningEncoderPort,
                ModuleConstants.kFrontLeftAngleZero,
                ModuleConstants.kModuleTurningGains,
                ModuleConstants.kModuleDriveGains);


        frontRight = new SwerveModule(
                "FR",
                ModuleConstants.kFrontRightDriveMotorPort,
                ModuleConstants.kFrontRightTurningMotorPort,
                ModuleConstants.kFrontRightTurningEncoderPort,
                ModuleConstants.kFrontRightAngleZero,
                ModuleConstants.kModuleTurningGains,
                ModuleConstants.kModuleDriveGains);


        rearLeft = new SwerveModule(
                "RL",
                ModuleConstants.kRearLeftDriveMotorPort,
                ModuleConstants.kRearLeftTurningMotorPort,
                ModuleConstants.kRearLeftTurningEncoderPort,
                ModuleConstants.kRearLeftAngleZero,
                ModuleConstants.kModuleTurningGains,
                ModuleConstants.kModuleDriveGains);


        rearRight = new SwerveModule(
                "RR",
                ModuleConstants.kRearRightDriveMotorPort,
                ModuleConstants.kRearRightTurningMotorPort,
                ModuleConstants.kRearRightTurningEncoderPort,
                ModuleConstants.kRearRightAngleZero,
                ModuleConstants.kModuleTurningGains,
                ModuleConstants.kModuleDriveGains);


        swervePosition = new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
        };


        gyro = new AHRS(NavXComType.kMXP_SPI);
        Timer.delay(1);
        gyro.zeroYaw();
        setInitialHeading(180.0); // keep your current setup
        setWheelsToZero();


        odometry = new SwerveDriveOdometry(
                DriveConstants.kDriveKinematics,
                getGyroRotation(),
                swervePosition);


        gyroTurnPidController = new ProfiledPIDController(
                DriveConstants.kGyroTurningGains.kP,
                DriveConstants.kGyroTurningGains.kI,
                DriveConstants.kGyroTurningGains.kD,
                new TrapezoidProfile.Constraints(
                        DriveConstants.kMaxTurningVelocityDegrees,
                        DriveConstants.kMaxTurningAcceleratonDegrees));


        gyroTurnPidController.enableContinuousInput(-180, 180);
        gyroTurnPidController.setTolerance(DriveConstants.kGyroTurnTolerance);


        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                gyro.getRotation2d(),
                swervePosition,
                new Pose2d());


        targetRotationDegrees = 0;


        SmartDashboard.putData("Field", field);


        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException("Failed to load robot config", e);
        }


        AutoBuilder.configure(
            this::getPose,
            this::resetOdometry,
            this::getRelativeChassisSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                    new PIDConstants(2.8, 0.0, 0.002),
                    new PIDConstants(4.5, 0.0, 0.001)
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }


    @Override
    public void periodic() {
        updateOdometry();


        SmartDashboard.putBoolean("Is Disabled", DriverStation.isDisabled());


        if (DriverStation.isDisabled()) {
            frontLeft.resetEncoders();
            frontRight.resetEncoders();
            rearLeft.resetEncoders();
            rearRight.resetEncoders();
            SmartDashboard.putString("Encoder Reset", "Resetting at " + Timer.getFPGATimestamp());
        }


        SmartDashboard.putNumber("FL Absolute", frontLeft.getAbsoluteHeading());
        SmartDashboard.putNumber("FR Absolute", frontRight.getAbsoluteHeading());
        SmartDashboard.putNumber("RL Absolute", rearLeft.getAbsoluteHeading());
        SmartDashboard.putNumber("RR Absolute", rearRight.getAbsoluteHeading());


        SmartDashboard.putNumber("FL Offset Check", frontLeft.getAbsoluteHeading() + frontLeft.angleZero);
        SmartDashboard.putNumber("FR Offset Check", frontRight.getAbsoluteHeading() + frontRight.angleZero);
        SmartDashboard.putNumber("RL Offset Check", rearLeft.getAbsoluteHeading() + rearLeft.angleZero);
        SmartDashboard.putNumber("RR Offset Check", rearRight.getAbsoluteHeading() + rearRight.angleZero);


        SmartDashboard.putNumber("Gyro yaw", getHeading());
        SmartDashboard.putNumber("Gyro pitch", gyro.getPitch());
        SmartDashboard.putNumber("Gyro roll", gyro.getRoll());
        SmartDashboard.putNumber("Gyro Turn Rate", getTurnRate());


        SmartDashboard.putNumber("FL Meters", frontLeft.getDistanceMeters());
        SmartDashboard.putNumber("FR Meters", frontRight.getDistanceMeters());
        SmartDashboard.putNumber("RL Meters", rearLeft.getDistanceMeters());
        SmartDashboard.putNumber("RR Meters", rearRight.getDistanceMeters());


        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Pose Rot", getPose().getRotation().getDegrees());


        SmartDashboard.putBoolean("LL Has Tag", LimelightHelpers.getTV(LimelightConstants.kLimelightName));
        SmartDashboard.putNumber("LL Tag ID", LimelightHelpers.getFiducialID(LimelightConstants.kLimelightName));
        SmartDashboard.putNumber("LL tx", LimelightHelpers.getTX(LimelightConstants.kLimelightName));
        SmartDashboard.putNumber("LL ta", LimelightHelpers.getTA(LimelightConstants.kLimelightName));


        Pose2d llPose = getLimelightFieldPose();
        SmartDashboard.putNumber("LL Pose X", llPose.getX());
        SmartDashboard.putNumber("LL Pose Y", llPose.getY());
        SmartDashboard.putNumber("LL Pose Rot", llPose.getRotation().getDegrees());


        SmartDashboard.putString("Alliance", DriverStation.getAlliance().map(Object::toString).orElse("Not Set"));
    }


    // region getters
    public Field2d getField() {
        return field;
    }


    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            rearLeft.getState(),
            rearRight.getState()
        };
    }


    public double getHeading() {
        return getGyroRotation().getDegrees();
    }


    private Rotation2d getGyroRotation() {
        return gyro.getRotation2d();
    }


    public double getHeading360() {
        double angle = getGyroRotation().getDegrees();
        angle = angle % 360.0;
        if (angle > 180.0) {
            angle -= 360.0;
        } else if (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }


    public double getRoll() {
        return gyro.getRoll();
    }


    public double getPitch() {
        return gyro.getPitch();
    }


    public double getTurnRate() {
        return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }


    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }


    public Pose2d getPoseEstimatorPose2d() {
        return poseEstimator.getEstimatedPosition();
    }


    public void resetOdometry(Pose2d pose) {
        swervePosition = new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
        };


        odometry.resetPosition(
                gyro.getRotation2d(),
                swervePosition,
                pose);


        poseEstimator.resetPosition(
                gyro.getRotation2d(),
                swervePosition,
                pose);


        field.setRobotPose(pose);
    }


    public ChassisSpeeds getRelativeChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
          frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState()
        );
    }


    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);


        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        rearLeft.setDesiredState(states[2]);
        rearRight.setDesiredState(states[3]);
    }


    public void setWheelsToZero() {
        SwerveModuleState zeroState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
        frontLeft.setDesiredState(zeroState);
        frontRight.setDesiredState(zeroState);
        rearLeft.setDesiredState(zeroState);
        rearRight.setDesiredState(zeroState);
    }


    // endregion


    // region setter
    public void lockWheels() {
        double rot = DriveConstants.kMaxRPM;


        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(0, 0, rot));


        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 0);


        setModuleStates(swerveModuleStates);
    }


    public void robotCentricDrive(double xSpeed, double ySpeed, double rot) {
        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(xSpeed, ySpeed, rot));
        setModuleStates(swerveModuleStates);
    }


    public void drive(double xSpeed, double ySpeed, double rot) {
        drive(xSpeed, ySpeed, rot, false, false);
    }


    public void drive(double xSpeed, double ySpeed, double rot, boolean isTurbo, boolean isSneak) {
        double maxSpeed;


        if (isSneak) {
            maxSpeed = DriveConstants.kMaxSneakMetersPerSecond;
        } else if (isTurbo) {
            maxSpeed = DriveConstants.kMaxTurboMetersPerSecond;
        } else {
            maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;
        }


        xSpeed *= maxSpeed;
        ySpeed *= maxSpeed;


        if (gyroTurning) {
            targetRotationDegrees += rot;
            rot = gyroTurnPidController.calculate(getHeading360(), targetRotationDegrees);
        } else {
            rot *= DriveConstants.kMaxRPM;
        }


        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, rot, getGyroRotation())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));


        setModuleStates(swerveModuleStates);
    }


    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.kMaxModuleSpeedMetersPerSecond);


        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }


    public void updateOdometry() {
        swervePosition = new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
        };


        odometry.update(getGyroRotation(), swervePosition);
        poseEstimator.update(getGyroRotation(), swervePosition);


        addLimelightVisionMeasurement();


        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }


    // IMPORTANT:
    // Always use WPI BLUE field coordinates for Field2d / pose estimator.
    // Do NOT swap to red based on alliance.
    private Pose2d getLimelightFieldPose() {
        String ll = LimelightConstants.kLimelightName;


        double[] botpose = LimelightHelpers.getBotPose_wpiRed(ll);


        if (botpose == null || botpose.length < 6) {
            return new Pose2d();
        }


        return new Pose2d(
                botpose[0],
                botpose[1],
                Rotation2d.fromDegrees(botpose[5]));
    }


    private void addLimelightVisionMeasurement() {
        String ll = LimelightConstants.kLimelightName;


        if (!LimelightHelpers.getTV(ll)) {
            return;
        }


        double ta = LimelightHelpers.getTA(ll);
        if (ta < kMinTargetArea) {
            return;
        }


        Pose2d visionPose = getLimelightFieldPose();


        // reject obvious bad pose
        if (visionPose.getX() == 0.0 && visionPose.getY() == 0.0) {
            return;
        }


        // reject pose if it is way outside expected field range
        if (visionPose.getX() < -1.0 || visionPose.getX() > 25.0
                || visionPose.getY() < -1.0 || visionPose.getY() > 15.0) {
            return;
        }


        // reject huge jumps
        double jump = visionPose.getTranslation()
                .getDistance(poseEstimator.getEstimatedPosition().getTranslation());


        if (jump > kMaxVisionDistanceJumpMeters) {
            return;
        }


        double latencyMs =
                LimelightHelpers.getLatency_Pipeline(ll)
                + LimelightHelpers.getLatency_Capture(ll);


        double timestampSeconds = Timer.getFPGATimestamp() - (latencyMs / 1000.0);


        poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
    }


    public void resetEncoders() {
        frontLeft.resetEncoders();
        rearLeft.resetEncoders();
        frontRight.resetEncoders();
        rearRight.resetEncoders();
    }


    public void zeroHeading() {
        gyro.reset();
    }


    public void setHeading(double heading) {
        gyro.setAngleAdjustment(heading - gyro.getYaw());
    }


    public Command toggleFieldCentric() {
        return runOnce(() -> {
            fieldRelative = !fieldRelative;
        });
    }


    public void setFieldCentric(boolean fieldCentric) {
        fieldRelative = fieldCentric;
    }


    public void stopMotors() {
        frontLeft.stopMotors();
        frontRight.stopMotors();
        rearLeft.stopMotors();
        rearRight.stopMotors();
    }


    public void setInitialHeading(double degrees) {
        gyro.setAngleAdjustment(degrees);
    }
    // endregion
}
