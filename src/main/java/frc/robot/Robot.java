package frc.robot;

//import edu.wpi.first.cameraserver.CameraServer;   //uncomment if we use cameras
//import edu.wpi.first.cscore.UsbCamera;  //uncomment if we use cameras
//import edu.wpi.first.math.controller.ElevatorFeedforward;
//import edu.wpi.first.net.PortForwarder;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private boolean wasDisabled = true; //I just added this 1/13/2026
  Thread m_visionThread1;
  //Thread m_visionThread2;
  private RobotContainer m_robotContainer;

 
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    SignalLogger.stop();   // Force disable Phoenix logging
    SignalLogger.enableAutoLogging(false);   // Force disable Phoenix logging
    m_robotContainer = new RobotContainer();

   //CameraServer.startAutomaticCapture();
    //  for (int port = 5800; port <= 5809; port++) {
		//  PortForwarder.add(port, "limelight.local", port);
		//  }
/**  <-- uncomment this if we are going to use USB camera
     m_visionThread1 = new Thread(
				() -> {
					 //Get the UsbCamera from CameraServer
					UsbCamera camera1 = CameraServer.startAutomaticCapture("CameraView", 0);
					camera1.setResolution(320, 480);
					camera1.setFPS(30);
					// UsbCamera camera2 = CameraServer.startAutomaticCapture("Cage View", 1);
					// camera2.setResolution(160, 120);
					// camera2.setFPS(15);


				});
		m_visionThread1.setDaemon(true);
		m_visionThread1.start();
 */

    DriverStation.silenceJoystickConnectionWarning(true);


    // m_visionThread2 = new Thread(() -> {
    //     UsbCamera camera2 = CameraServer.startAutomaticCapture("Cage View", 1);
    //     camera2.setResolution(160, 120);
    //     camera2.setFPS(15);
    //     });
    // m_visionThread2.setDaemon(true);
    // m_visionThread2.start();

  }



  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (DriverStation.isEnabled() && wasDisabled) {
        RobotContainer.driveSubsystem.zeroHeading();
        wasDisabled = false;
    } else if (DriverStation.isDisabled()) {
        wasDisabled = true;
    }
}

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
