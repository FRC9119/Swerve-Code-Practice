package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimbConstants.*;

public class ElevatorClimber extends SubsystemBase{
    private final TalonFX thisWayAndThatWay;

    public ElevatorClimber() {

        thisWayAndThatWay = new TalonFX(CLIMB_MOTOR_ID);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = MAX_CLIMB_CURRENT;
        thisWayAndThatWay.getConfigurator().apply(config);
    
        SmartDashboard.putNumber("Elevating climb value", MAX_CLIMB_CURRENT);

    }

    public void L1climb() {
        try {
        thisWayAndThatWay.set(1);
        TimeUnit.SECONDS.sleep(CLIMB_CYCLE_TIME);
        thisWayAndThatWay.set(-1);
        } catch (InterruptedException e) {
System.out.println(e);
        }
    }

    public void LCLIMB_CYCLE_TIMEclimb() {
        try{
        thisWayAndThatWay.set(1);
        TimeUnit.SECONDS.sleep(CLIMB_CYCLE_TIME);
        thisWayAndThatWay.set(-1);
        TimeUnit.SECONDS.sleep(CLIMB_CYCLE_TIME);
        thisWayAndThatWay.set(1);
        TimeUnit.SECONDS.sleep(CLIMB_CYCLE_TIME);
        thisWayAndThatWay.set(-1);
        TimeUnit.SECONDS.sleep(CLIMB_CYCLE_TIME);
        thisWayAndThatWay.set(1);
        TimeUnit.SECONDS.sleep(CLIMB_CYCLE_TIME);
        thisWayAndThatWay.set(-1);
        } catch (InterruptedException e) {
System.out.println(e);
        }
    }
}
