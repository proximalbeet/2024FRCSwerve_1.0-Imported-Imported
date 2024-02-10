// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ActivateShooterCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final DoubleSupplier leftPercent, rightPercent;

  /** Creates a new ActivateShooterCommand. */
  public ActivateShooterCmd(ShooterSubsystem shooterSubsystem, DoubleSupplier leftPercent, DoubleSupplier rightPercent) {
    this.shooterSubsystem = shooterSubsystem;
    this.leftPercent = leftPercent;
    this.rightPercent = rightPercent;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.ActivateShooter(leftPercent.getAsDouble(), rightPercent.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.DeactivateShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
