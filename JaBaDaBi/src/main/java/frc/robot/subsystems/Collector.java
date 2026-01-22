package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {

    private SparkMax dCollectorMotor;
    private SparkMax iCollectorMotor;

    private SparkMaxConfig dCollectMotorConfig;
    private SparkMaxConfig iCollectMotorConfig;
    private SparkClosedLoopController iCollectorM;
    private SparkClosedLoopController dCollectorM;

    public Collector() {

        dCollectorMotor = new SparkMax(34, MotorType.kBrushless);
        iCollectorMotor = new SparkMax(35, MotorType.kBrushless);

        dCollectMotorConfig = new SparkMaxConfig();
        dCollectMotorConfig.idleMode(IdleMode.kBrake);
        dCollectMotorConfig.smartCurrentLimit(40);
        dCollectMotorConfig.inverted(false);
        iCollectMotorConfig = new SparkMaxConfig();
        iCollectMotorConfig.follow(34, true);

        dCollectorMotor.configure(dCollectMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        iCollectorMotor.configure(iCollectMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        iCollectorM = iCollectorMotor.getClosedLoopController();
        dCollectorM = iCollectorMotor.getClosedLoopController();
    }

    public Command collect(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            setIntakeSetPoint(setPoint.getAsDouble());
            setDeploySetPoint(setPoint.getAsDouble());
        }, this);
    }

    public void setIntakeSetPoint(double setPoint) {

        iCollectorM.setSetpoint(setPoint, ControlType.kPosition);

    }

    public void setDeploySetPoint(double setPoint) {

        dCollectorM.setSetpoint(setPoint, ControlType.kPosition);

    }

}
