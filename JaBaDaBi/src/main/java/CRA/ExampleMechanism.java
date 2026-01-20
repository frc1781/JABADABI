package CRA;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleMechanism extends SubsystemBase{
    // Example motor and config
    private final SparkMax m_mechanism = new SparkMax(0, MotorType.kBrushless);
    private final SparkMaxConfig m_config = new SparkMaxConfig();

    // PID Controller and PID Tuning. Make sure that they both have the same values.
    private PIDController m_pidController = new PIDController(0.01, 0, 0);
    private final PIDTuning m_pidTuning = new PIDTuning("ExampleMechanism", 0.01, 0, 0, 0);
    private double[] pidValues;

    // Setpoint for the PID controller, can be adjusted as needed
    private final double setpoint = 10.0;

    public ExampleMechanism() {
        // Configure the SparkMax motor controller
        m_config.idleMode(SparkMaxConfig.IdleMode.kBrake);
        m_config.smartCurrentLimit(30);
        m_config.inverted(false);

        m_mechanism.configure(m_config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void periodic() {
        pidValues = m_pidTuning.getPID(); // Get PID values from the shuffleboard
        m_pidController.setPID(
            pidValues[0], // kP
            pidValues[1], // kI
            pidValues[2]  // kD
        ); // Set PID controller values

        // Calculate the output from the PID controller to reach the set point and set it to the motor.
        m_mechanism.set(m_pidController.calculate(m_mechanism.getEncoder().getPosition(), setpoint));
    }
}
