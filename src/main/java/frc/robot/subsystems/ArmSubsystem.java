// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  
  public CANSparkMax ArmMotor;
  PIDController armPID;
  public double desiredArmPosition;

  public ArmSubsystem(){
        ArmMotor = new CANSparkMax(9, MotorType.kBrushless);

         desiredArmPosition = ArmConstants.COLLAPSED;

    ArmMotor.setNeutralMode(NeutralMode.Brake);
    ArmMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    armPID = new PIDController(
      0.000005,
      0.0000000001,
      0
    );

    ArmMotor.config_kP(0, 0.000005);
  }
   
  @Override
  public void periodic() {
 

   SmartDashboard.putNumber("armPosition", ArmMotor.getEncoder().getPosition());

    armPID.setSetpoint(desiredArmPosition);
    double calculation = armPID.calculate(ArmMotor.getEncoder().getPosition(), desiredArmPosition);
    ArmMotor.set(ControlMode.PercentOutput, MathUtil.clamp(calculation, -1, 1));

  }

  public void driveArm(double speed) {
    ArmMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getArmPosition() { return ArmMotor.getEncoder().getPosition();}
}

 // /** Creates a new Arm. */
  // public void SetArmSpeed(double percent) {
  //   ArmMotor.set(percent);
  //   SmartDashboard.putNumber("arm power (%)", percent);
  //   SmartDashboard.putNumber("arm motor current (amps)", ArmMotor.getOutputCurrent());
  //   SmartDashboard.putNumber("arm motor temperature (C)", ArmMotor.getMotorTemperature());
  // }
