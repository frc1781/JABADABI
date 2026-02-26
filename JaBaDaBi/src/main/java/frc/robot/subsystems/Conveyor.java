package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

public class Conveyor extends SubsystemBase {
    private final SparkMax motor = new SparkMax(Constants.Conveyor.MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();
    private double motorPower;

    public Conveyor() {
        motorPower = 0;
        config.idleMode(SparkBaseConfig.IdleMode.kCoast);
        config.smartCurrentLimit(30);
        config.inverted(true);
        motor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic() {
        motor.set(motorPower);
        Logger.recordOutput("Conveyor/motorPower", motorPower);
        System.out.println();
    }

    public Command loadFuel(BooleanSupplier fuelPresent) {
        return new RunCommand(() -> {
            motorPower = fuelPresent.getAsBoolean() ? 1 : 0;
        }, this);
    }

    public Command idle() {
        return new RunCommand(() -> {
            motorPower = 0;
        }, this);
    }
}