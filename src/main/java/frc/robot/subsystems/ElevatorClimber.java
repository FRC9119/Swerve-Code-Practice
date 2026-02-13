package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class ElevatorClimber extends SubsystemBase{
    private final TalonFX thisWayAndThatWay;

    public ElevatorClimber() {

        thisWayAndThatWay = new TalonFX(MAX_CLIMB_CURRENT);

        SmartDashboard.putNumber("Elevating climb value", MAX_CLIMB_CURRENT);
    }

    public void climb() {
        thisWayAndThatWay.setVoltage(SmartDashboard.getNumber("Elevating climb value", MAX_CLIMB_CURRENT));
    }
}
