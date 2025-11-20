// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
  /** Creates a new OuttakeSubsystem. */
  
  TalonFX outTakeMotor;

  public OuttakeSubsystem() {
    outTakeMotor = new TalonFX(12); 
  }

  public void motorOutTake() {
    outTakeMotor.setVoltage(10);
  }

  public void motorMoveBack(){
    outTakeMotor.setVoltage(-2);
  }

  public void stopmotorOutTake() {
    outTakeMotor.setVoltage(0);

  }
  public double getPos(){
    return outTakeMotor.getPosition().getValueAsDouble();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber( "Outtake Position", getPos());
    SmartDashboard.putNumber("outtakeMotorVel", outTakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("outtakeMotorVoltage", outTakeMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("OuttakeCurrentSupply", outTakeMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("OuttakeCurrentStator", outTakeMotor.getStatorCurrent().getValueAsDouble());
  }
}
