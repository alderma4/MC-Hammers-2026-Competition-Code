package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private final Spark blinkin;

    private boolean intakeRunning = false;
    private boolean shooterAtTargetRPM = false;

    public LEDSubsystem() {
        blinkin = new Spark(0); // PWM port 0 (where Blinkin is plugged in)
    }

    public void setIntakeRunning(boolean running) {
        intakeRunning = running;
    }

    public void setShooterAtTargetRPM(boolean atTarget) {
        shooterAtTargetRPM = atTarget;
    }

    @Override
    public void periodic() {

        if (DriverStation.isDisabled()) {
            blinkin.set(.53); // red + white animation (strobe)
        }
        else if (shooterAtTargetRPM) {
            blinkin.set(0.77); // green
        }
        else if (intakeRunning) {
            blinkin.set(0.67); // yellow/gold
        }
        else {
            blinkin.set(0.61); // red
        }
    }
}