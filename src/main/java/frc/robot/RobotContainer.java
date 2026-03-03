// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.FuelConstants.USE_SHOOTER_LIMELIGHT;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberInABox;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
         // kSpeedAt12Volts desired top speed
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        // 3/4 of a rotation per second max angular velocity
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        // Add a 10% deadband
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        // Request to drive and face hub
        private final SwerveRequest.FieldCentricFacingAngle targetHub = new SwerveRequest.FieldCentricFacingAngle()
                        .withHeadingPID(10, 0, 0)
                        // Add a 10% deadband
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
        // Init logging
        private final Telemetry logger = new Telemetry(MaxSpeed);
        // Driver Controller
        private final CommandXboxController joystick = new CommandXboxController(0);
        // Operator Controller
        private final CommandXboxController operatorController = new CommandXboxController(1);
        // Init Subsystems
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem(drivetrain);
        public final ClimberInABox climbSubsystem = new ClimberInABox(drivetrain);
        // Init auto
        public final Auto auto = new Auto(drivetrain, ballSubsystem);

        public RobotContainer() {
                // Make limelight webpage available on roboRIO IP
                LimelightHelpers.setupPortForwardingUSB(0);
                // Publish values to the dashboard
                Dashboard.publish();

                configureBindings();
        }

        private void configureBindings() {
                // Add joystick controll to swerve request
               // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-Math.atan(joystick.getRawAxis(1) * MaxSpeed))
                                                .withVelocityY(-Math.atan(joystick.getRawAxis(0) * MaxSpeed))
                                                .withRotationalRate(-joystick.getRawAxis(2) * MaxAngularRate)));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final SwerveRequest.Idle idle = new SwerveRequest.Idle();
                final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
                /*
                 * While Y is held AND when constant USE_SHOOTER_LIMELIGHT is true, target hub.
                 * Otherwise run default drive SwerveRequest
                 */
                // TODO: use ctre brake request
                operatorController.y().whileTrue(ballSubsystem.spinUpCommand()
                                .until(() -> ballSubsystem.launchBang.atSetpoint())
                                .andThen(ballSubsystem.launchCommand()).finallyDo(() -> ballSubsystem.stop()))

                                .whileTrue(drivetrain.applyRequest(() -> {
                                        if (USE_SHOOTER_LIMELIGHT){
                                                //TODO: ask what button this should be
                                                if(joystick.rightTrigger().getAsBoolean()) return brake;
                                                return targetHub.withTargetDirection(
                                                                new Rotation2d(Targeting.getRadiansBetweenRobotAndHub(
                                                                                drivetrain.getState().Pose)))
                                                                .withVelocityX(-Math.atan(
                                                                                joystick.getRawAxis(1) * MaxSpeed * .8))
                                                                .withVelocityY(-Math.atan(
                                                                                joystick.getRawAxis(0) * MaxSpeed * .8));
                                        }
                                        else
                                                return drive.withVelocityX(-Math.atan(joystick.getRawAxis(1) * MaxSpeed))
                                                .withVelocityY(-Math.atan(joystick.getRawAxis(0) * MaxSpeed))
                                                .withRotationalRate(-joystick.getRawAxis(2) * MaxAngularRate);

                                }));
                // Run outtake (called eject()) periodically while B is pressed
                operatorController.b()
                                .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(),
                                                () -> ballSubsystem.stop()));
                // Run intake() periodically while X is pressed
                operatorController.x()
                                .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(),
                                                () -> ballSubsystem.stop()));
                // Run unclog() periodically while X is pressed
                operatorController.a()
                                .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.unclog(),
                                                () -> ballSubsystem.stop()));
                // TODO: figure out what buttons to map
                operatorController.button(0).whileTrue(climbSubsystem.runEnd(() -> climbSubsystem.climb(), () -> climbSubsystem.stop()));
                operatorController.button(0).whileTrue(climbSubsystem.runEnd(() -> climbSubsystem.release(), () -> climbSubsystem.stop()));
                
                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset the field-centric heading on left bumper press
                joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
                        
                // zero gyro yaw on right bumper press
                joystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.getPigeon2().setYaw(0)));
                // give logs to drivetrain
                drivetrain.registerTelemetry(logger::telemeterize);
        }

}
