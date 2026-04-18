// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.FuelConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.Targeting;

public class CANFuelSubsystem extends SubsystemBase {
  private final TalonFX feederRoller;
  private final TalonFX launcherRoller;
  private final TalonFX launcherSecondary;
  private final TalonFX intakeRoller;
  private final CommandSwerveDrivetrain drivetrain;
  public final PIDController launchPID;
  public final SimpleMotorFeedforward launchFF = new SimpleMotorFeedforward(0.14789, 0.11835, 0);
  public final SysIdRoutine sysIdFlywheelRoutine;
  public double setpointRPS = DEFAULT_LAUNCH_RPS;

  private final NetworkTable fuelSubsytemTable = NetworkTableInstance.getDefault().getTable("FuelSubsystem");
  private final DoublePublisher flywheelTargetPub = fuelSubsytemTable.getDoubleTopic("flywheelTarget").publish();

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // create brushed motors for each of the motors on the launcher mechanism
    launcherRoller = new TalonFX(LAUNCHER_MOTOR_ID_1);
    launcherSecondary = new TalonFX(LAUNCHER_MOTOR_ID_2);
    launcherSecondary.setControl(new Follower(LAUNCHER_MOTOR_ID_1, MotorAlignmentValue.Aligned));

    feederRoller = new TalonFX(FEEDER_MOTOR_ID);
    intakeRoller = new TalonFX(INTAKE_MOTOR_ID);

    // Create bang-bang controller
    launchPID = new PIDController(0.181, 0, 0);
    // Add tolerance (amount of error that is still considered correct)
    launchPID.setTolerance(LAUNCH_TOLERANCE);

    // Add sysId routine to get PID/FF values for flywheel
    sysIdFlywheelRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> launcherRoller.setVoltage(output.magnitude()),
            null,
            this));
  }

  @Override
  public void periodic() {
    // publish shooter target speed to network tables and write to log file
    flywheelTargetPub.set(setpointRPS);
    SignalLogger.writeDouble("FuelSubsystem/flywheelTarget", setpointRPS);
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederRoller.setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeRoller
        .setVoltage(-SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // Method to unclog the shooter just in case a fuel is stuck
  public void unclog() {
    feederRoller.setVoltage(6);
    intakeRoller
        .setVoltage(8);
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederRoller
        .setVoltage(-SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeRoller
        .setVoltage(SmartDashboard.getNumber("Intaking launcher roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for launching.
  public void launch(boolean useTargeting) {
    if (useTargeting)
      setpointRPS = Targeting.getTargetRPS(drivetrain.getPose());
    launcherRoller
        .setVoltage(launchFF.calculate(setpointRPS)
            + launchPID.calculate(launcherRoller.getVelocity().getValueAsDouble(), setpointRPS));

    intakeRoller.setVoltage(-6);
    feederRoller.setVoltage(-SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));

  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.setVoltage(0);
    intakeRoller.setVoltage(0);
    launcherRoller.setVoltage(0);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    setpointRPS = Targeting.getTargetRPS(drivetrain.getPose());
    launcherRoller
        .setVoltage(
            launchFF.calculate(setpointRPS) + launchPID.calculate(launcherRoller.getVelocity().getValueAsDouble(), setpointRPS));
  }

  // Command factories to turn the spinUp method into a command that requires this
  // subsystem

  public Command spinUpCommand() {
    return this.run(this::spinUp).finallyDo(this::stop);
  }

  public Command intakeWithUnclogCommand() {
    return Commands.sequence(intakeCommand().withTimeout(1), ejectCommand().withTimeout(.1));
  }

  public Command intakeCommand() {
    return this.run(this::intake).finallyDo(this::stop);
  }

  public Command ejectCommand() {
    return this.run(this::eject).finallyDo(this::stop);
  }

  public Command launchCommand() {
    return this.run(() -> launch(true)).finallyDo(this::stop);
  }

  public Command launchWithoutTargeting(double rps) {
    return this.run(() -> launch(false)).alongWith(Commands.run(() -> setpointRPS = rps)).finallyDo(this::stop);
  }

  public Command unclogCommand() {
    return this.run(this::unclog).finallyDo(this::stop);
  }

  public Command stopCommand() {
    return this.run(this::stop);
  }

}
