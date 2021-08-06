// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.commands.ArmDown;
import frc.robot.commands.ArmUp;
import frc.robot.commands.ZeroArmSensor;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final Arm m_arm = new Arm();

  // The driver's controller
  public static XboxController m_driverController =
      new XboxController(Constants.OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driverController.getRawAxis(0),
                    m_driverController.getRawAxis(1)),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  JoystickButton btn1 = new JoystickButton(m_driverController, 1);
  JoystickButton btn2 = new JoystickButton(m_driverController, 2);
  JoystickButton btn3 = new JoystickButton(m_driverController, 3);
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kBumperRight.value)
        .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
        .whenReleased(() -> m_robotDrive.setMaxOutput(1));
    btn1.whenPressed(new ArmUp(m_arm));
    btn2.whenPressed(new ArmDown(m_arm));
    btn3.whenPressed(new ZeroArmSensor(m_arm));
    
  }

  public DriveSubsystem getRobotDrive() {
    return m_robotDrive;
  }

  /** Zeros the outputs of all subsystems. */
  public void zeroAllOutputs() {
    m_robotDrive.tankDriveVolts(0, 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DriveConstants.kDriveKinematics,
            7);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(3, 5.7, new Rotation2d(180)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(
    //             new Translation2d(2, 3),
    //             new Translation2d(4, 1)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(6, 3, new Rotation2d(0)),
    //         // Pass config
    //         config);

    // RamseteCommand ramseteCommand =
    //     new RamseteCommand(
    //         Robot.getAutonTraj(),
    //         m_robotDrive::getPose,
    //         new RamseteController(
    //             Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
    //         new SimpleMotorFeedforward(
    //             Constants.DriveConstants.ksVolts,
    //             Constants.DriveConstants.kvVoltSecondsPerMeter,
    //             Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
    //         Constants.DriveConstants.kDriveKinematics,
    //         m_robotDrive::getWheelSpeeds,
    //         new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
    //         new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
    //         // RamseteCommand passes volts to the callback
    //         m_robotDrive::tankDriveVolts,
    //         m_robotDrive);

    RamseteController ramsetecontroller = new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta);
    SimpleMotorFeedforward motorff = new SimpleMotorFeedforward(
                                                                Constants.DriveConstants.ksVolts,
                                                                Constants.DriveConstants.kvVoltSecondsPerMeter,
                                                                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter);


    RamseteCommand ramseteCommandRev =
        new RamseteCommand(
            Robot.getAutonTraj(),
            m_robotDrive::getPose,
            ramsetecontroller,
            motorff,
            Constants.DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);

    RamseteCommand ramseteCommandFwd =
        new RamseteCommand(
            Robot.trajectoryfwd,
            m_robotDrive::getPose,
            ramsetecontroller,
            motorff,
            Constants.DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);

    // Reset odometry to starting pose of trajectory.
    
    m_robotDrive.resetOdometry(Robot.getAutonTraj().getInitialPose());

    // Run path following command, then stop at the end.

    
    return 
    ramseteCommandRev
    .andThen(ramseteCommandFwd)
    .andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }
}
