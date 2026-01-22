package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Hopper extends SubsystemBase{

    private SparkMax hopper;


    public Hopper() {
    hopper= new SparkMax(40, null);
    

    leftshooterConfig= new SparkFlexConfig();
    leftshooterConfig.idleMode(IdleMode.kCoast);
    leftshooterConfig.smartCurrentLimit(40);
    leftshooterConfig.inverted(false);
    rightshooterConfig= new SparkFlexConfig();
    rightshooterConfig.follow(40, true);

    leftshooter.configure(leftshooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightshooter.configure(rightshooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftmController= leftshooter.getClosedLoopController();


}


public Command shoot(DoubleSupplier setPoint) {
    return new RunCommand(() -> {
        setMotorSetPoint(setPoint.getAsDouble());
    }
    , this);
}


public void setMotorSetPoint(double setPoint) {


leftmController.setSetpoint(setPoint, ControlType.kVelocity);

    }


}