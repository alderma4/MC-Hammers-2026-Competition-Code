// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


  //You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands 
public class UpdateLights extends Command {
  private final LightSubsystem lightSubsystem;
  private final ShooterSubsystem intakeSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  // Creates a new UpdateLights. 

  public UpdateLights(LightSubsystem lightSubsystem, ShooterSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
      this.lightSubsystem = lightSubsystem;
      this.intakeSubsystem = intakeSubsystem;
      this.elevatorSubsystem = elevatorSubsystem;

   

    // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(lightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lightSubsystem.setRed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeSubsystem.isIntakeOn()) {
      lightSubsystem.setGreen();
    } else {
      double height = elevatorSubsystem.elevEncoderState();
      if (height >= LightSubsystem.High_Threshold) {
        lightSubsystem.setWhite();
      } else if (height >= LightSubsystem.Mid_Threshold) {
        lightSubsystem.setPurple();
      } else if (height >= LightSubsystem.Low_Threshold) {
        lightSubsystem.setYellow();
      } else {
        lightSubsystem.setBlue();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lightSubsystem.setRed();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}*/
