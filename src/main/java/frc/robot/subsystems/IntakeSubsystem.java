// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkFlex m_intakeMotor = new SparkFlex(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);
  private final SparkMax m_intakePivotMotor = new SparkMax(IntakeConstants.kIntakePivotMotorId, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake() {
    m_intakeMotor.getClosedLoopController().setSetpoint(1500, ControlType.kVelocity);
  }

  public void stopIntake() {
    m_intakeMotor.getClosedLoopController().setSetpoint(0, ControlType.kVelocity);
  }

  public void setIntakePivotSpeed(double speed) {
    m_intakePivotMotor.set(speed);
  }
}
