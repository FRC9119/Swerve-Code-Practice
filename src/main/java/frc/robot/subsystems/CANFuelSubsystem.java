// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.FuelConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Targeting;

public class CANFuelSubsystem extends SubsystemBase {
  private final WPI_TalonSRX feederRoller;
  private final TalonFX launcherRoller;
  private final TalonFX intakeRoller;
  private final CommandSwerveDrivetrain drivetrain;
  public BangBangController launchBang;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // create brushed motors for each of the motors on the launcher mechanism
    launcherRoller = new TalonFX(LAUNCHER_MOTOR_ID);
    feederRoller = new WPI_TalonSRX(FEEDER_MOTOR_ID);
    intakeRoller = new TalonFX(INTAKE_MOTOR_ID);
    // create config for feeder CIM and apply it
    TalonSRXConfiguration feederConfig = new TalonSRXConfiguration();
    feederConfig.peakCurrentLimit = FEEDER_MOTOR_CURRENT_LIMIT;
    feederRoller.configAllSettings(feederConfig);
    // Create bang-bang controller and add a setpoint (goal)
    launchBang = new BangBangController();
    launchBang.setSetpoint(DEFAULT_LAUNCH_RPM);
    // Add tolerance (amount of error that is still considered correct)
    launchBang.setTolerance(LAUNCH_TOLERANCE);
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederRoller.setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeRoller
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // Method to unclog the shooter just in case a fuel is stuck
  public void unclog() {
    feederRoller.setVoltage(-6);
    intakeRoller
        .setVoltage(8);
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederRoller
        .setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeRoller
        .setVoltage(SmartDashboard.getNumber("Intaking launcher roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    launchBang.setSetpoint(Targeting.getTargetRPM(drivetrain.getState().Pose));
    intakeRoller.setVoltage(-6);
    feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    launcherRoller
        .set(launchBang.calculate(launcherRoller.getVelocity().getValueAsDouble() * 60));
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    launcherRoller.set(0);
    intakeRoller.set(0);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    launchBang.setSetpoint(Targeting.getTargetRPM(drivetrain.getState().Pose));

    launcherRoller
        .set(launchBang.calculate(launcherRoller.getVelocity().getValueAsDouble() * 60));
  }

  // Command factories to turn the spinUp method into a command that requires this subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  public Command intakeCommand() {
    return this.run(() -> intake());
  }

  public Command launchCommand() {
    return this.run(() -> launch());
  }

  public Command unclogCommand() {
    return this.run(() -> unclog());
  }

  public Command stopCommand() {
    return this.run(() -> stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
