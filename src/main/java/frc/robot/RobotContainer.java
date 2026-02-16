// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.FuelConstants.FULL_FIELD_X;
import static frc.robot.Constants.FuelConstants.FULL_FIELD_Y;
import static frc.robot.Constants.FuelConstants.HUB_X_COORD;
import static frc.robot.Constants.FuelConstants.HUB_Y_COORD;
import static frc.robot.Constants.FuelConstants.USE_SHOOTER_LIMELIGHT;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.Constants.FuelConstants.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final PIDController shootAimPID = new PIDController(SHOOT_AIM_KP, SHOOT_AIM_KI, SHOOT_AIM_KD);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem(drivetrain);
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        NamedCommands.registerCommand("spinUP",ballSubsystem.spinUpCommand());
        NamedCommands.registerCommand("launchCommand", ballSubsystem.launchCommand());
        NamedCommands.registerCommand("intake", ballSubsystem.intakeCommand());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
private double getRadiansBetweenRobotAndHub (){
        Translation2d bluePose = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? (new Translation2d(FULL_FIELD_X,FULL_FIELD_Y)).minus(drivetrain.getState().Pose.getTranslation()) : drivetrain.getState().Pose.getTranslation();
        return Math.atan2(bluePose.getX()-HUB_Y_COORD,bluePose.getX()-HUB_X_COORD);
}
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(Math.atan(-joystick.getRawAxis(0) * MaxSpeed * .8)) // Drive
                                                                                                                      // forward
                                                                                                                      // with
                                                                                                                      // negative
                                                                                                                      // Y
                                                                                                                      // (forward)
                        .withVelocityY(Math.atan(joystick.getRawAxis(1) * MaxSpeed * .8)) // Drive left with negative X
                                                                                          // (left)
                        .withRotationalRate(operatorController.y().getAsBoolean() && USE_SHOOTER_LIMELIGHT ? shootAimPID.calculate(drivetrain.getState().Pose.getRotation().getRadians(), getRadiansBetweenRobotAndHub()) : -joystick.getRawAxis(2) * MaxAngularRate) // Drive counterclockwise with
                                                                                      // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                
        operatorController.y().whileTrue(ballSubsystem.spinUpCommand().until(()->ballSubsystem.launchBang.atSetpoint())
                .andThen(ballSubsystem.launchCommand()).finallyDo(() -> ballSubsystem.stop()));
        operatorController.b()
                .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));
        operatorController.x()
                .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
        operatorController.a()
                .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.unclog(), () -> ballSubsystem.stop()));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

}
