package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private SparkFlex leftshooter;
    private SparkFlex rightshooter;

    private SparkFlexConfig leftshooterConfig;
    private SparkFlexConfig rightshooterConfig;

    private SparkClosedLoopController leftmController;
    private double lastRPM = 0.0;
    private boolean recovering = false;
    private double boostEndTime = 0.0;
    private double targetRPM = 4500.0;
    private static final double BOOST_DURATION_SEC = 0.5;
    private static final double BOOST_OUTPUT = 1.0;


    public Shooter() {
        leftshooter = new SparkFlex(Constants.Shooter.SHOOTER_1_CAN_ID, MotorType.kBrushless);
        rightshooter = new SparkFlex(Constants.Shooter.SHOOTER_2_CAN_ID, MotorType.kBrushless);

        leftshooterConfig = new SparkFlexConfig();
        leftshooterConfig.idleMode(IdleMode.kCoast);
        leftshooterConfig.smartCurrentLimit(40);
        leftshooterConfig.inverted(false);
        rightshooterConfig = new SparkFlexConfig();
        rightshooterConfig.follow(Constants.Shooter.SHOOTER_1_CAN_ID, true);
        leftshooterConfig.closedLoop.pid(0.0013, 0, 0.001).feedForward.kV(0.000157);

        leftshooter.configure(leftshooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightshooter.configure(rightshooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        leftmController = leftshooter.getClosedLoopController();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/position", leftshooter.getEncoder().getPosition());
        Logger.recordOutput("Shooter/velocity", leftshooter.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/voltage", leftshooter.getBusVoltage()*leftshooter.getAppliedOutput());
        double currentRPM = leftshooter.getEncoder().getVelocity();
    double rpmDelta = currentRPM - lastRPM;  // negative when dropping

    // Tune this threshold based on your shooter
    boolean shotDetected = rpmDelta < -200.0 && !recovering;

    if (shotDetected) {
        startRecoveryBoost();
    }

    if (recovering && Timer.getFPGATimestamp() > boostEndTime) {
        endRecoveryBoost();
    }

    lastRPM = currentRPM;
        
    }

    public void startShooter() {
    // Normal SmartVelocity control
    leftmController.setSetpoint(targetRPM, ControlType.kVelocity);
    recovering = false;
}

private void startRecoveryBoost() {
    recovering = true;
    boostEndTime = Timer.getFPGATimestamp() + BOOST_DURATION_SEC;
    leftshooter.set(BOOST_OUTPUT);  // open-loop boost
}

private void endRecoveryBoost() {
    recovering = false;
    // Hand control back to SmartVelocity
    leftmController.setSetpoint(targetRPM, ControlType.kVelocity);
}

    public Command shoot(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            setMotorSetPoint(setPoint.getAsDouble());
            Logger.recordOutput("Shooter/requestedvelocity", setPoint);
            System.out.println("Aaron is cool Antonio is not");
        }, this);
    }

    public void setMotorSetPoint(double setPoint) {
        leftmController.setSetpoint(setPoint, ControlType.kVelocity);
    }

}




