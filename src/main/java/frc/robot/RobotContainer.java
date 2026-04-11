package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import java.lang.reflect.Method;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SpindexerConstants;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.PreloadShoot;
import frc.robot.commands.LLDriveToTarget;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeFlipperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {

  // ---------------- SUBSYSTEMS ----------------
  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static final FeederSubsystem feederSubsystem = new FeederSubsystem();
  public static final SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static final IntakeFlipperSubsystem intakeFlipperSubsystem = new IntakeFlipperSubsystem();
  public static final LEDSubsystem ledSubsystem = new LEDSubsystem();

  // Limelight
  public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  // ---------------- LED STATE TRACKING ----------------
  private boolean intakeLedActive = false;

  // ---------------- JOYSTICKS ----------------
  private final CommandJoystick driveJoystick =
      new CommandJoystick(OperatorConstants.kDriveJoystickPort);

  private final CommandJoystick turnJoystick =
      new CommandJoystick(OperatorConstants.kTurnJoystickPort);

  private final Joystick Leftjoy = new Joystick(OperatorConstants.kDriveJoystickPort);
  private final Joystick Rightjoy = new Joystick(OperatorConstants.kTurnJoystickPort);

  // ---------------- XBOX CONTROLLER ----------------
  @SuppressWarnings("unused")
  private final XboxController Controller1 = new XboxController(Constants.XBOXCONTROLLER_ID);
  private final CommandXboxController ccontroller = new CommandXboxController(Constants.XBOXCONTROLLER_ID);

  // Face buttons
  Trigger XboxButton1 = ccontroller.button(1); // A
  Trigger XboxButton2 = ccontroller.button(2); // B
  Trigger XboxButton3 = ccontroller.button(3); // X
  Trigger XboxButton4 = ccontroller.button(4); // Y

  // Bumpers
  Trigger XboxButton5 = ccontroller.button(5); // LB
  Trigger XboxButton6 = ccontroller.button(6); // RB

  // Stick buttons
  Trigger leftStickBtn = ccontroller.leftStick();
  Trigger rightStickBtn = ccontroller.rightStick();

  // Start / Back
  Trigger startBtn = ccontroller.start();
  Trigger backBtn = ccontroller.back();

  // D-Pad (POV)
  Trigger povUp = ccontroller.povUp();
  Trigger povRight = ccontroller.povRight();
  Trigger povDown = ccontroller.povDown();
  Trigger povLeft = ccontroller.povLeft();

  // ---------------- OTHER BUTTONS ----------------
  public JoystickButton button12 = new JoystickButton(Leftjoy, Constants.BUTTON_12_ID);
  public JoystickButton rightButton11 = new JoystickButton(Rightjoy, Constants.BUTTON_11_ID);

  // NEW: left Logitech joystick stick press (button 10)
  public JoystickButton leftButton11 = new JoystickButton(Leftjoy, 11);

  // Flight stick trigger (Left joystick button 1)
  public JoystickButton driveTrigger = new JoystickButton(Leftjoy, Constants.BUTTON_1_ID);

  // Keep this if you were using it before
  public JoystickButton lTrigger = new JoystickButton(Leftjoy, Constants.BUTTON_1_ID);

  // ---------------- AUTO ----------------
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    // Register NamedCommands for PathPlanner
    NamedCommands.registerCommand(
        "PreloadShootClose",
        PreloadShoot.close(shooterSubsystem, feederSubsystem, spindexerSubsystem));

    NamedCommands.registerCommand(
        "PreloadShootMedium",
        PreloadShoot.medium(shooterSubsystem, feederSubsystem, spindexerSubsystem));

    NamedCommands.registerCommand(
        "PreloadShootFar",
        PreloadShoot.far(shooterSubsystem, feederSubsystem, spindexerSubsystem));
    NamedCommands.registerCommand(
      "CenterPreload", 
      PreloadShoot.centerAuto(shooterSubsystem, feederSubsystem, spindexerSubsystem));
    NamedCommands.registerCommand(
      "LeftPreload",
       PreloadShoot.leftAuto(shooterSubsystem, feederSubsystem, spindexerSubsystem));
    NamedCommands.registerCommand(
      "RightPreload",
       PreloadShoot.rightAuto(shooterSubsystem, feederSubsystem, spindexerSubsystem));

    NamedCommands.registerCommand(
        "EnableAutoSlowMode",
        Commands.runOnce(() -> driveSubsystem.setAutoSpeedMultiplier(0.35), driveSubsystem));

    NamedCommands.registerCommand(
        "DisableAutoSlowMode",
        Commands.runOnce(() -> driveSubsystem.resetAutoSpeedMultiplier(), driveSubsystem));

    // ---------------- AUTON INTAKE / FLIPPER COMMANDS ----------------

    // Flip intake out once
    NamedCommands.registerCommand(
        "FlipIntakeOut",
        Commands.runOnce(
            () -> intakeFlipperSubsystem.moveToOutPosition(),
            intakeFlipperSubsystem));

    // Flip intake in once
    NamedCommands.registerCommand(
        "FlipIntakeIn",
        Commands.runOnce(
            () -> intakeFlipperSubsystem.moveToInPosition(),
            intakeFlipperSubsystem));

    // Run intake continuously until timeout
    NamedCommands.registerCommand(
        "AutonIntake",
        Commands.run(
            () -> intakeSubsystem.intakeIn(),
            intakeSubsystem)
        .withTimeout(5.0)
        .finallyDo(interrupted -> intakeSubsystem.stop()));

    // Timed intake command
    NamedCommands.registerCommand(
        "AutonIntake2Sec",
        Commands.run(
            () -> intakeSubsystem.intakeIn(),
            intakeSubsystem)
        .withTimeout(2.0)
        .finallyDo(interrupted -> intakeSubsystem.stop()));

    // Intake out / spit out
    NamedCommands.registerCommand(
        "AutonIntakeOut",
        Commands.run(
            () -> intakeSubsystem.intakeOut(),
            intakeSubsystem)
        .withTimeout(1.0)
        .finallyDo(interrupted -> intakeSubsystem.stop()));

    // Flip intake out, then intake for 2 seconds
    NamedCommands.registerCommand(
        "FlipOutAndIntake",
        Commands.sequence(
            Commands.runOnce(
                () -> intakeFlipperSubsystem.moveToOutPosition(),
                intakeFlipperSubsystem),
            Commands.waitSeconds(0.25),
            Commands.run(
                () -> intakeSubsystem.intakeIn(),
                intakeSubsystem)
            .withTimeout(2.0)
            .finallyDo(interrupted -> intakeSubsystem.stop())));

    // Assume the flipper is physically fully IN when robot starts up.
    intakeFlipperSubsystem.zeroAtInPosition();
    intakeFlipperSubsystem.moveToInPosition();

    // ---------------- DEFAULT LED UPDATE ----------------
    ledSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              ledSubsystem.setIntakeRunning(intakeLedActive);
              ledSubsystem.setShooterAtTargetRPM(getShooterAtTargetForLED());
            },
            ledSubsystem));

    configureBindings();

    // ---------------- MANUAL AUTO CHOOSER ----------------
    // These are loaded lazily so robot code can still boot cleanly.
    autoChooser = new SendableChooser<>();

    autoChooser.setDefaultOption(
        "Center Single Shoot",
        createLazyAuto("Center Single Shoot"));

    autoChooser.addOption(
        "Left Double Shoot",
        createLazyAuto("Left Double Shoot"));

    autoChooser.addOption(
        "Right Double Shoot",
        createLazyAuto("Right Double Shoot"));

    Shuffleboard.getTab("Driver").add("Auto Chooser", autoChooser);
    SmartDashboard.putData("Auto Choices", autoChooser);

    // Shuffleboard flipper position
    Shuffleboard.getTab("Driver")
        .addNumber("Intake Flipper Pos (rot)", () -> intakeFlipperSubsystem.getPosition());

    // Limelight values on Shuffleboard
    Shuffleboard.getTab("Driver").addBoolean("LL Has Target", () -> limelightSubsystem.hasTarget());
    Shuffleboard.getTab("Driver").addNumber("LL tx", () -> limelightSubsystem.getTX());
    Shuffleboard.getTab("Driver").addNumber("LL ta", () -> limelightSubsystem.getTA());

    // ---------------- LIVE FIELD WIDGET ----------------
    Shuffleboard.getTab("Driver")
        .add("Robot Field", driveSubsystem.getField())
        .withWidget(BuiltInWidgets.kField)
        .withPosition(6, 0)
        .withSize(6, 4)
        .withProperties(Map.of(
            "Robot width", 0.9,
            "Robot length", 0.9));
  }

  private Command createLazyAuto(String autoName) {
    return Commands.defer(
        () -> {
          try {
            System.out.println("Loading auto: " + autoName);
            return AutoBuilder.buildAuto(autoName);
          } catch (Exception e) {
            System.out.println("FAILED TO LOAD AUTO: " + autoName);
            e.printStackTrace();
            return Commands.none();
          }
        },
        Set.of());
  }

  private void configureBindings() {

    // ---------------- DRIVE ----------------
    button12.onTrue(driveSubsystem.toggleFieldCentric());

    // Keep your original reset encoders on left trigger
    lTrigger.whileTrue(new InstantCommand(() -> driveSubsystem.resetEncoders()));

    rightButton11.onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));

    // LIMELIGHT DRIVE-TO-TARGET while holding flight stick trigger
    driveTrigger.whileTrue(new LLDriveToTarget(driveSubsystem, limelightSubsystem));

    // ---------------- SHOOTER PRESETS (D-PAD) ----------------
    povDown.onTrue(new InstantCommand(() -> shooterSubsystem.setMode(ShooterMode.CLOSE)));
    povRight.onTrue(new InstantCommand(() -> shooterSubsystem.setMode(ShooterMode.MEDIUM)));
    povUp.onTrue(new InstantCommand(() -> shooterSubsystem.setMode(ShooterMode.FAR)));

    // ---------------- FEEDER ----------------
    XboxButton2.whileTrue(new RunCommand(() -> feederSubsystem.runForward(), feederSubsystem));
    XboxButton2.onFalse(new InstantCommand(() -> feederSubsystem.stop(), feederSubsystem));

    XboxButton4.whileTrue(new RunCommand(() -> feederSubsystem.runReverse(), feederSubsystem));
    XboxButton4.onFalse(new InstantCommand(() -> feederSubsystem.stop(), feederSubsystem));

    // ---------------- SHOOTER ----------------
    XboxButton3.whileTrue(new RunCommand(() -> shooterSubsystem.runSelectedRPM(), shooterSubsystem));
    XboxButton3.onFalse(new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem));

    // NEW: left Logitech joystick stick press (button 10) = opposite direction, full power, no ramp
    leftButton11.whileTrue(new RunCommand(() -> shooterSubsystem.runOppositeFullPower(), shooterSubsystem));
    leftButton11.onFalse(new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem));

    // ---------------- SPINDEXER ----------------
    XboxButton4.whileTrue(new RunCommand(() -> spindexerSubsystem.runAtRPM(SpindexerConstants.kFeedRPM), spindexerSubsystem));
    XboxButton4.onFalse(new InstantCommand(() -> spindexerSubsystem.stop(), spindexerSubsystem));

    XboxButton2.whileTrue(new RunCommand(() -> spindexerSubsystem.runReverseRPM(), spindexerSubsystem));
    XboxButton2.onFalse(new InstantCommand(() -> spindexerSubsystem.stop(), spindexerSubsystem));

    // ---------------- INTAKE (ROLLER) ----------------
    XboxButton1.onTrue(new InstantCommand(() -> intakeLedActive = true));
    XboxButton1.whileTrue(new RunCommand(() -> intakeSubsystem.intakeIn(), intakeSubsystem));
    XboxButton1.onFalse(new InstantCommand(() -> {
      intakeSubsystem.stop();
      intakeLedActive = false;
    }, intakeSubsystem));

    povLeft.onTrue(new InstantCommand(() -> intakeLedActive = true));
    povLeft.whileTrue(new RunCommand(() -> intakeSubsystem.intakeOut(), intakeSubsystem));
    povLeft.onFalse(new InstantCommand(() -> {
      intakeSubsystem.stop();
      intakeLedActive = false;
    }, intakeSubsystem));

    // ---------------- INTAKE FLIPPER ----------------
    XboxButton5.onTrue(new InstantCommand(() -> intakeFlipperSubsystem.moveToOutPosition(), intakeFlipperSubsystem));
    XboxButton6.onTrue(new InstantCommand(() -> intakeFlipperSubsystem.moveToInPosition(), intakeFlipperSubsystem));

    // Start/Back: manual override while held
    backBtn.whileTrue(new RunCommand(() -> intakeFlipperSubsystem.manualFlipOut(), intakeFlipperSubsystem));
    backBtn.onFalse(new InstantCommand(() -> intakeFlipperSubsystem.stopManual(), intakeFlipperSubsystem));

    startBtn.whileTrue(new RunCommand(() -> intakeFlipperSubsystem.manualFlipIn(), intakeFlipperSubsystem));
    startBtn.onFalse(new InstantCommand(() -> intakeFlipperSubsystem.stopManual(), intakeFlipperSubsystem));

    // ---------------- DEFAULT DRIVE ----------------
    driveSubsystem.setDefaultCommand(
        new DriveCommand(
            driveSubsystem,
            () -> -driveJoystick.getY(),
            () -> -driveJoystick.getX(),
            () -> -turnJoystick.getX()));
  }

  /**
   * Tries common shooter "at speed" method names without forcing you to rename your subsystem.
   * If none of these methods exist yet, LEDs will stay red until you send me ShooterSubsystem.java
   * and I can wire the exact method in.
   */
  private boolean getShooterAtTargetForLED() {
    String[] methodNames = {
        "atTargetRPM",
        "isAtTargetRPM",
        "atSpeed",
        "isAtSpeed",
        "shooterAtTargetRPM",
        "isShooterAtTargetRPM"
    };

    for (String methodName : methodNames) {
      try {
        Method method = shooterSubsystem.getClass().getMethod(methodName);
        Object result = method.invoke(shooterSubsystem);
        if (result instanceof Boolean) {
          return (Boolean) result;
        }
      } catch (Exception e) {
        // Try next method name
      }
    }

    return false;
  }

  public Command getAutonomousCommand() {
    Command selected = autoChooser.getSelected();
    return selected != null ? selected : Commands.none();
  }
}