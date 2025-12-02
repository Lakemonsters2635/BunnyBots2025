// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX m_intakeMotor1;
  private final TalonFX m_intakeMotor2;


  public IntakeSubsystem() {
    m_intakeMotor1 = new TalonFX(Constants.INTAKE_MOTOR_ID1);
    m_intakeMotor2 = new TalonFX(Constants.INTAKE_MOTOR_ID2);
  
    
  }

  public void intake(){
    m_intakeMotor1.setVoltage(Constants.INTAKE_MOTOR_VOLTAGE);
    m_intakeMotor2.setVoltage(-Constants.INTAKE_MOTOR_VOLTAGE);
  }

  public void stopIntake(){
    m_intakeMotor1.setVoltage(0);
    m_intakeMotor2.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
