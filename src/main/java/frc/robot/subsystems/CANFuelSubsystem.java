// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.FuelConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);

    TalonSRXConfiguration feederConfig = new TalonSRXConfiguration();
    feederConfig.peakCurrentLimit = FEEDER_MOTOR_CURRENT_LIMIT;
    
    launchBang = new BangBangController();
    launchBang.setSetpoint(DEFAULT_LAUNCH_RPM);
    launchBang.setTolerance(LAUNCH_TOLERANCE);
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederRoller.setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeRoller
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }

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
    launchBang.setSetpoint(getTargetRPM());
    feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    launcherRoller
           .set(launchBang.calculate(launcherRoller.getVelocity().getValueAsDouble()*60));
                  System.out.println(launchBang.getError());

  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    launcherRoller.set(0);
    intakeRoller.set(0);
  }
  public double getTargetRPM(){    if (!USE_SHOOTER_LIMELIGHT) return DEFAULT_LAUNCH_RPM;

    Translation2d coordinates = drivetrain.getState().Pose.getTranslation();
    Translation2d blueCoordinates = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? (new Translation2d(FULL_FIELD_X,FULL_FIELD_Y)).minus(coordinates) : coordinates;
    return AT_HUB_LAUNCH_RPM + LIMELIGHT_RPM_KP * (blueCoordinates.getDistance(new Translation2d(HUB_X_COORD, HUB_Y_COORD))- MIN_HUB_DISTANCE);
  }
  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    launchBang.setSetpoint(getTargetRPM());
    intakeRoller.setVoltage(-7);
    feederRoller
        .setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
   launcherRoller
           .set(launchBang.calculate(launcherRoller.getVelocity().getValueAsDouble()*60));
          System.out.println(launchBang.getError());
          }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }
  public Command intakeCommand(){
    return this.run(() -> intake());
  }
  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launch());
  }

  public Command unclogCommand() {
    return this.run(() -> unclog());
  }
  public Command stopCommand(){
    return this.run(() -> stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
