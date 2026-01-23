package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

    private SparkMax hopperMotor;

    private SparkMaxConfig hopperConfig;

    private SparkClosedLoopController hopperController;

    public Hopper() {
        hopperMotor = new SparkMax(40, MotorType.kBrushless);

        hopperConfig = new SparkMaxConfig();
        hopperConfig.idleMode(IdleMode.kCoast);
        hopperConfig.smartCurrentLimit(40);
        hopperConfig.inverted(false);

        hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hopperController = hopperMotor.getClosedLoopController();
    }

    public Command runHopper(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            setMotorSetPoint(setPoint.getAsDouble());
        }, this);
    }

    public void setMotorSetPoint(double setPoint) {
        hopperController.setSetpoint(setPoint, ControlType.kVelocity);
    }

}