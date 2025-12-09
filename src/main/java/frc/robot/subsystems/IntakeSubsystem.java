// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_intakeMotor1;
  private final SparkMax m_intakeMotor2;

  private final SparkMaxConfig m_intakeMotor1Config;
  private final SparkMaxConfig m_intakeMotor2Config;


  public IntakeSubsystem() {
    m_intakeMotor1 = new SparkMax(Constants.INTAKE_MOTOR_TOP_ID, MotorType.kBrushless);
    m_intakeMotor2 = new SparkMax(Constants.INTAKE_MOTOR_BOTTOM_ID, MotorType.kBrushless);

    m_intakeMotor1Config = new SparkMaxConfig();
    m_intakeMotor2Config = new SparkMaxConfig();

    m_intakeMotor1.configure(
      m_intakeMotor1Config, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
    
    m_intakeMotor2.configure(
      m_intakeMotor2Config, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
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
