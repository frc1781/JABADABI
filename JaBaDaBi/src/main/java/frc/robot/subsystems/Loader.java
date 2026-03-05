package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Robots;

public class Loader extends SubsystemBase {

    private TalonFX motor;
    private TalonFXConfiguration loaderConfig;
    private double motorPower;

    public Loader() {
        if (Robot.getRobot() != Robots.SAVITAR) {
            return;
        }    
    
        motor = new TalonFX(Constants.Loader.MOTOR_CAN_ID);

        loaderConfig = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(40))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.CounterClockwise_Positive));

        motor.getConfigurator().apply(loaderConfig);
    }

    public void periodic() {
        Logger.recordOutput("Loader/motorPower", motorPower);
        if (Robot.getRobot() != Robots.SAVITAR) {
            return;
        }  
        
        motor.set(motorPower);
    }

    public Command runLoader(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            motorPower = setPoint.getAsDouble();
        }, this);
    }

    public Command idle() {
        return new RunCommand(() -> {
            motorPower = 0;
        }, this);
    }
}