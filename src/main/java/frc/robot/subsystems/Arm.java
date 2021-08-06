// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Instrum;
import frc.robot.RobotContainer;
import frc.robot.sim.PhysicsSim;

public class Arm extends SubsystemBase {
  public final TalonSRX m_arm = new WPI_TalonSRX(Constants.OIConstants.kArmMotorID);
  
	/* Used to build string throughout loop */
  StringBuilder _sb = new StringBuilder();
  
	/** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing = 0;

	/** save the last Point Of View / D-pad value */
	int _pov = -1;
  /** Creates a new Arm. */
  public Arm() {
    
		PhysicsSim.getInstance().addTalonSRX(m_arm, 0.75, 5100, false);
		/* Factory default hardware to prevent unexpected behavior */
		m_arm.configFactoryDefault();
		/* Configure Sensor Source for Pirmary PID */
		m_arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.ArmConstants.kPIDLoopIdx,
				Constants.ArmConstants.kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
      m_arm.configNeutralDeadband(0.001, Constants.ArmConstants.kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		m_arm.setSensorPhase(false);
		m_arm.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		m_arm.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.ArmConstants.kTimeoutMs);
		m_arm.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.ArmConstants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		m_arm.configNominalOutputForward(0, Constants.ArmConstants.kTimeoutMs);
		m_arm.configNominalOutputReverse(0, Constants.ArmConstants.kTimeoutMs);
		m_arm.configPeakOutputForward(1, Constants.ArmConstants.kTimeoutMs);
		m_arm.configPeakOutputReverse(-1, Constants.ArmConstants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		m_arm.selectProfileSlot(Constants.ArmConstants.kSlotIdx, Constants.ArmConstants.kPIDLoopIdx);
		m_arm.config_kF(Constants.ArmConstants.kSlotIdx, Constants.ArmConstants.kGains.kF, Constants.ArmConstants.kTimeoutMs);
		m_arm.config_kP(Constants.ArmConstants.kSlotIdx, Constants.ArmConstants.kGains.kP, Constants.ArmConstants.kTimeoutMs);
		m_arm.config_kI(Constants.ArmConstants.kSlotIdx, Constants.ArmConstants.kGains.kI, Constants.ArmConstants.kTimeoutMs);
		m_arm.config_kD(Constants.ArmConstants.kSlotIdx, Constants.ArmConstants.kGains.kD, Constants.ArmConstants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		m_arm.configMotionCruiseVelocity(3000, Constants.ArmConstants.kTimeoutMs);
		m_arm.configMotionAcceleration(3000, Constants.ArmConstants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		m_arm.setSelectedSensorPosition(0, Constants.ArmConstants.kPIDLoopIdx, Constants.ArmConstants.kTimeoutMs);
    
  }

  public void resetArm() {    
    m_arm.setSelectedSensorPosition(0);
  }

  public void setArm(double setpoint) {
    double targetpos = setpoint * 4096 * 10.0;
    
    m_arm.set(ControlMode.MotionMagic, targetpos);

    /* Append more signals to print when in speed mode */
    _sb.append("\terr:");
    _sb.append(m_arm.getClosedLoopError(Constants.ArmConstants.kPIDLoopIdx));
    _sb.append("\ttrg:");
    _sb.append(targetpos);
  }
  @Override
  public void periodic() {
    XboxController _joy = RobotContainer.m_driverController;
		/* Get gamepad axis - forward stick is positive */
		// double leftYstick = -1.0 * _joy.getY(Hand.kRight); /* left-side Y for Xbox360Gamepad */
		// double rghtYstick = -1.0 * _joy.getX(Hand.kRight); /* right-side Y for Xbox360Gamepad */
		// if (Math.abs(leftYstick) < 0.10) { leftYstick = 0; } /* deadband 10% */
		// if (Math.abs(rghtYstick) < 0.10) { rghtYstick = 0; } /* deadband 10% */

		/* Get current Talon SRX motor output */
		double motorOutput = m_arm.getMotorOutputPercent();

		/* Prepare line to print */
		_sb.append("\tOut%:");
		_sb.append(motorOutput);
		_sb.append("\tVel:");
		_sb.append(m_arm.getSelectedSensorVelocity(Constants.ArmConstants.kPIDLoopIdx));

		/**
		 * Peform Motion Magic when Button 1 is held, else run Percent Output, which can
		 * be used to confirm hardware setup.
		 */
		// if (_joy.getRawButton(1)) {
			/* Motion Magic */

			/* 4096 ticks/rev * 10 Rotations in either direction */
			// double targetPos = rghtYstick * 4096 * 10.0;
			// m_arm.set(ControlMode.MotionMagic, targetPos);

			// /* Append more signals to print when in speed mode */
			// _sb.append("\terr:");
			// _sb.append(m_arm.getClosedLoopError(Constants.ArmConstants.kPIDLoopIdx));
			// _sb.append("\ttrg:");
			// _sb.append(targetPos);
		// } else {
		// 	/* Percent Output */

		// 	m_arm.set(ControlMode.PercentOutput, leftYstick);
		// }
		// if (_joy.getRawButton(2)) {
		// 	/* Zero sensor positions */
		// 	m_arm.setSelectedSensorPosition(0);
		// }

		int pov = _joy.getPOV();
		if (_pov == pov) {
			/* no change */
		} else if (_pov == 180) { // D-Pad down
			/* Decrease smoothing */
			_smoothing--;
			if (_smoothing < 0)
				_smoothing = 0;
        m_arm.configMotionSCurveStrength(_smoothing);

			System.out.println("Smoothing is set to: " + _smoothing);
		} else if (_pov == 0) { // D-Pad up
			/* Increase smoothing */
			_smoothing++;
			if (_smoothing > 8)
				_smoothing = 8;
        m_arm.configMotionSCurveStrength(_smoothing);

			System.out.println("Smoothing is set to: " + _smoothing);
		}
		_pov = pov; /* save the pov value for next time */

		/* Instrumentation */
		Instrum.Process(m_arm, _sb);
  }

  @Override 
  public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
    
  }

}
