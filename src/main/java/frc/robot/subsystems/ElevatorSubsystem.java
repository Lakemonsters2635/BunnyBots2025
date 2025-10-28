// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  public static SparkMax m_elevatorMotor;
  public static PIDController m_elevatorController;

  public static SparkMaxConfig m_elevatorConfig;
  double fb=0, ff=0;//TODO: figure out ff value when we get robot

  double targetPos = 0;


  public ElevatorSubsystem() {
    m_elevatorController = new PIDController(0, 0, 0); //TODO: configure PID values later
    m_elevatorMotor = new SparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    m_elevatorConfig = new SparkMaxConfig();

    m_elevatorConfig.idleMode(IdleMode.kBrake);
    m_elevatorConfig.inverted(false);

    m_elevatorMotor.configure(
    m_elevatorConfig,
    SparkBase.ResetMode.kResetSafeParameters, 
    SparkBase.PersistMode.kPersistParameters
    );
  }

  public void setTargetPos(double target){
    targetPos = target;
  }

  public void setVoltage(double voltage){
    m_elevatorMotor.setVoltage(voltage);
  }

  public double getEncoderValue(){
    return m_elevatorMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    fb = m_elevatorController.calculate(getEncoderValue(), targetPos);
    fb = MathUtil.clamp(fb, -2, 2); //TODO: FIX THESE PLS LATER
    setVoltage(ff+fb);
  }
}
