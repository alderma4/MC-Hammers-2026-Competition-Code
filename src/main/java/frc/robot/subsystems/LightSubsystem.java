// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LightSubsystem extends SubsystemBase {
  // Creates a new LightsSubsystem. 

  private final Spark spark;
  private final ShooterSubsystem intake;
  private final ElevatorSubsystem elevator;

  // Color Values for the LEDs
  private static final double GREEN = 0.77; // Intake on
  private static final double RED = 0.61; // Intake off
  private static final double BLUE = 0.65; // Level 1
  private static final double YELLOW = 0.69; // Level 2
  private static final double PURPLE = 0.91; // Level 3
  private static final double WHITE = 0.93; // Level 4

  public static final double High_Threshold = 140;
  public static final double Mid_Threshold = 80;
  public static final double Low_Threshold = 30;

  private String currentColor;

  public LightSubsystem(ShooterSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
    spark = new Spark(0);
    this.intake = intakeSubsystem;
    this.elevator = elevatorSubsystem;
    SmartDashboard.putString("Light Color", "unknown");
    
  }

  public void updateLights() {
    double valueToSet;
    if (intake.isIntakeOn()) {
      valueToSet = GREEN;
      currentColor = "Green";
    } else { 
      double height = elevator.elevEncoderState();
      if (height >= High_Threshold) {
        valueToSet = BLUE;
        currentColor = "Blue";
      } else if (height >= Mid_Threshold) {
        valueToSet = PURPLE;
        currentColor = "Purple";
      } else if (height >= Low_Threshold) {
        valueToSet = YELLOW;
        currentColor = "Yellow";
      } else {
        valueToSet = WHITE;
        currentColor = "White";
      }
    }
    spark.set(valueToSet);
    SmartDashboard.putString("Light Color", currentColor);
    SmartDashboard.putNumber("Light Value", valueToSet);
  }


public void setGreen() {
  spark.set(GREEN);
  currentColor = "Green";
  SmartDashboard.putString("Light Color", currentColor);
  SmartDashboard.putNumber("Light Value", GREEN);
}

public void setRed() {
  spark.set(RED);
  currentColor = "Red";
  SmartDashboard.putString("Light Color", currentColor);
  SmartDashboard.putNumber("Light Value", RED);
}

public void setBlue() {
  spark.set(BLUE);
  currentColor = "Blue";
  SmartDashboard.putString("Light Color", currentColor);
  SmartDashboard.putNumber("Light Value", BLUE);
} 

public void setYellow() {
  spark.set(YELLOW);
  currentColor = "Yellow";
  SmartDashboard.putString("Light Color", currentColor);
  SmartDashboard.putNumber("Light Value", YELLOW);
} 

public void setPurple() {
  spark.set(PURPLE);
  currentColor = "Purple";
  SmartDashboard.putString("Light Color", currentColor);
  SmartDashboard.putNumber("Light Value", PURPLE);
}

public void setWhite() {
  spark.set(WHITE);
  currentColor = "White";
  SmartDashboard.putString("Light Color", currentColor);
  SmartDashboard.putNumber("Light Value", WHITE);
}

} */
