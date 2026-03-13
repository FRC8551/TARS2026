// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UserConfig;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkFlex m_intakeMotor = new SparkFlex(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);
  private final SparkMax m_pivotMotor = new SparkMax(IntakeConstants.kPivotMotorId, MotorType.kBrushless);

  private boolean m_pivotCalibrated = false;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig.closedLoop.feedForward.kV(0.00019);
    intakeConfig.closedLoop.pid(0.0001, 0, 0);
    intakeConfig.inverted(true);
    intakeConfig.smartCurrentLimit(80);

    m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig pivotConfig = new SparkFlexConfig();
    pivotConfig.closedLoop.pid(0.04, 0, 0);
    pivotConfig.inverted(true);

    m_pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(IntakeConstants.kSlash + "Intake RPM", m_intakeMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber(IntakeConstants.kSlash + "Intake Output", m_intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber(IntakeConstants.kSlash + "Pivot Position", m_pivotMotor.getEncoder().getPosition());
    SmartDashboard.putNumber(IntakeConstants.kSlash + "Pivot Output Current", m_pivotMotor.getOutputCurrent());
    SmartDashboard.putBoolean(IntakeConstants.kSlash + "Pivot Calibrated", m_pivotCalibrated);

    if (!m_pivotCalibrated) {
      if (m_pivotMotor.getOutputCurrent() > IntakeConstants.kPivotStallCurrentThreshold) {
        m_pivotMotor.set(0);
        m_pivotMotor.getEncoder().setPosition(0);
        m_pivotCalibrated = true;
      } else {
        m_pivotMotor.set(-0.1);
      }
    }
  }

  public Command runIntake() {
    return run(() -> {
      if (m_intakeMotor.getEncoder().getVelocity() < IntakeConstants.kIntakeRPM - IntakeConstants.kIntakeRPMTolerance
          && UserConfig.getBeansModeEnabled()) {
        // Beans Mode 😎
        m_intakeMotor.setVoltage(12);
      } else {
        // Burger Mode 🍔
        m_intakeMotor.getClosedLoopController().setSetpoint(IntakeConstants.kIntakeRPM, ControlType.kVelocity);
      }
    });
  }

  public Command stopIntake() {
    return run(() -> m_intakeMotor.set(0));
  }

  public Command setIntakePivotSpeed(double speed) {
    return run(() -> m_pivotMotor.set(speed));
  }

  public Command setPivotPosition(double position) {
    return run(() -> {
      if (m_pivotCalibrated) {
        m_pivotMotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
      }
    });
  }
}
