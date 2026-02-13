package frc.robot.subsystems;

java.util.concurrent.TimeUnit;

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

    public void L1climb() {
        thisWayAndThatWay.set(1);
        TimeUnit.SECONDS.sleep(30);
        thisWayAndThatWay.set(-1);
    }

    public void L3climb() {
        thisWayAndThatWay.set(1);
        TimeUnit.SECONDS.sleep(30);
        thisWayAndThatWay.set(-1);
        TimeUnit.SECONDS.sleep(30);
        thisWayAndThatWay.set(1);
        TimeUnit.SECONDS.sleep(30);
        thisWayAndThatWay.set(-1);
        TimeUnit.SECONDS.sleep(30);
        thisWayAndThatWay.set(1);
        TimeUnit.SECONDS.sleep(30);
        thisWayAndThatWay.set(-1);
    }
}
