// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This is a sample program to demonstrate the use of state-space classes in robot simulation. This
 * robot has a flywheel, elevator, arm and differential drivetrain, and interfaces with the sim
 * GUI's {@link edu.wpi.first.wpilibj.simulation.Field2d} class.
 */
public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  String trajectoryRevJSON = "output/TestReverse.wpilib.json";
  String trajectoryFwdJSON = "output/SmallForward.wpilib.json";
  public static Trajectory trajectoryrev = new Trajectory();
  public static Trajectory trajectoryfwd = new Trajectory();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    try {
      Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryRevJSON);
      trajectoryrev = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryRevJSON, ex.getStackTrace());
    }
    try {
      Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFwdJSON);
      trajectoryfwd = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryFwdJSON, ex.getStackTrace());
    }


    m_robotContainer.m_robotDrive.resetOdometry(Robot.getAutonTraj().getInitialPose());

    // Flush NetworkTables every loop. This ensures that robot pose and other values
    // are sent during every loop iteration.
    setNetworkTablesFlushEnabled(true);
  }

  public static Trajectory getAutonTraj() {
    return trajectoryrev;
  }

  @Override
  public void simulationPeriodic() {
    // Here we calculate the battery voltage based on drawn current.
    // As our robot draws more power from the battery its voltage drops.
    // The estimated voltage is highly dependent on the battery's internal
    // resistance.
    double drawCurrent = m_robotContainer.getRobotDrive().getDrawnCurrentAmps();
    double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawCurrent);
    RoboRioSim.setVInVoltage(loadedVoltage);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.zeroAllOutputs();
  }
}
