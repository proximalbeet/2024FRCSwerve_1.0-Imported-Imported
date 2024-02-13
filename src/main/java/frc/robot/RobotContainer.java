// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.Console;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ActivateShooterCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final Joystick driverJoystick1 = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick driverJoystick2 = new Joystick(OIConstants.kDriverControllerPort2);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /* 
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoystick1.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoystick1.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick1.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !driverJoystick1.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
      ));
      */

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoystick1.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoystick1.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick1.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !driverJoystick1.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
      ));

        System.out.println(!driverJoystick1.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx));
     
      
        // Configure the button bindings
    configureButtonBindings();
  }

  
  private void configureButtonBindings() {
    //Button on the Joystick that gets pressed to move the swerve drive
    //new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
    new JoystickButton(driverJoystick1, Constants.OIConstants.PresetHeadingZero).onTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading() , swerveSubsystem));

    new JoystickButton(driverJoystick2, Constants.OIConstants.PresetButtonIndexA).onTrue(Commands.runOnce(() -> armSubsystem.driveArm(Constants.ArmConstants.armPos) , armSubsystem));

    new JoystickButton(driverJoystick2, Constants.OIConstants.PresetButtonIndexB)
                              .whileTrue(new ActivateShooterCmd(
                                      shooterSubsystem, 
                                      () -> Constants.ShooterConstants.leftPower, 
                                      () -> Constants.ShooterConstants.rightPower));

    new JoystickButton(driverJoystick2, Constants.OIConstants.PresetButtonIndexC)
                              .whileTrue(new ActivateShooterCmd(
                                      shooterSubsystem, 
                                      () -> Constants.ShooterConstants.leftPowerN, 
                                      () -> Constants.ShooterConstants.rightPowerN));
  }



  
  public Command getAutonomousCommand() {
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);

    //- - - - - Add Auton movements here - - - - -
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d()), 
      List.of(
            new Translation2d(1, 0),
            new Translation2d(1, -1)

      ),
      new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
      trajectoryConfig
    );

    // 3.Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerContraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

      // 5.Add some init and wrap-up, and return everything
      return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> swerveSubsystem.stopModules())
    );
  }
}
