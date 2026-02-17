package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dashboard;

import static frc.robot.Constants.ClimbConstants.*;

public class ElevatorClimber extends SubsystemBase{
    private final TalonFX thisWayAndThatWay;
private final SwerveRequest.FieldCentric drive;
private final CommandSwerveDrivetrain drivetrain;
private final PIDController alignX = new PIDController(ALIGN_X_KP,0,0);     
      private final  PIDController alignY = new PIDController(ALIGN_Y_KP,0,0);
      private final  PIDController alignTheta = new PIDController(ALIGN_THETA_KP,0,0);
    public ElevatorClimber(SwerveRequest.FieldCentric drive, CommandSwerveDrivetrain drivetrain, Dashboard dash) {
        this.drive = drive;
        this.drivetrain = drivetrain;
        thisWayAndThatWay = new TalonFX(CLIMB_MOTOR_ID);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = MAX_CLIMB_CURRENT;
        thisWayAndThatWay.getConfigurator().apply(config);
    }
    public void align(Alignment a){
        Pose2d goalPosition;
        Pose2d currentPosition = drivetrain.getState().Pose;
        switch (a) {
            case Left: 
                goalPosition = new Pose2d();
            case Right:
                goalPosition = new Pose2d();
            default:
                goalPosition = new Pose2d();
        }
        
        alignX.setSetpoint(goalPosition.getX());
        alignY.setSetpoint(goalPosition.getY());
        alignTheta.setSetpoint(goalPosition.getRotation().getRadians());


        drivetrain.applyRequest(() -> drive.withVelocityX(alignX.calculate(currentPosition.getX())).withVelocityY(currentPosition.getY()).withRotationalRate(currentPosition.getRotation().getRadians()));
        
    
    }
public enum Alignment {
    Left, Right
}
    public void L1climb(Alignment a) {
        align(a);
        L1climb();
    }
    public void L1climb(){
        try {
        thisWayAndThatWay.set(1);
        TimeUnit.SECONDS.sleep(CLIMB_CYCLE_TIME);
        thisWayAndThatWay.set(-1);
        } catch (InterruptedException e) {
            System.out.println(e);
        }
    }

    public void L3climb(Alignment a) {
        align(a);
        L3climb();
    }
    public void L3climb(){
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
