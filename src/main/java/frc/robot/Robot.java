package frc.robot;

//import edu.wpi.first.cameraserver.CameraServer;   //uncomment if we use cameras
//import edu.wpi.first.cscore.UsbCamera;  //uncomment if we use cameras
//import edu.wpi.first.math.controller.ElevatorFeedforward;
//import edu.wpi.first.net.PortForwarder;
import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  Thread m_visionThread1;
  //Thread m_visionThread2;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    StatusLogger.disableAutoLogging();
    SignalLogger.stop();
    SignalLogger.enableAutoLogging(false);
    m_robotContainer = new RobotContainer();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}