package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

public class Conveyor extends SubsystemBase {
    private final SparkMax motor = new SparkMax(Constants.Conveyor.MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();

    public Conveyor() {
        config.idleMode(SparkBaseConfig.IdleMode.kCoast);
        config.smartCurrentLimit(Constants.Conveyor.CURRENT_LIMIT);
        motor.configure(config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }
    
    public Command clearCoral(BooleanSupplier hasCoralToClear) {
        return new RunCommand(() -> {motor.set(hasCoralToClear.getAsBoolean() ? 0.5 : 0);}, this);
    }         
}