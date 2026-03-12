// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HammerToolbox.JoystickUtils;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {

  private final DriveSubsystem driveSubsystem;
  private final Supplier<Double> driveY;
  private final Supplier<Double> driveX;
  private final Supplier<Double> rotateX;

  public DriveCommand(
      DriveSubsystem driveSubsystem,
      Supplier<Double> driveY,
      Supplier<Double> driveX,
      Supplier<Double> rotateX) {

    this.driveSubsystem = driveSubsystem;
    this.driveY = driveY;
    this.driveX = driveX;
    this.rotateX = rotateX;

    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    if (RobotState.isTeleop()) {
      driveSubsystem.drive(
          JoystickUtils.processJoystickInput(driveY.get()),
          JoystickUtils.processJoystickInput(driveX.get()),
          JoystickUtils.processJoystickInput(rotateX.get()));
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}