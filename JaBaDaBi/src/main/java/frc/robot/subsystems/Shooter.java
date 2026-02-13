package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import CRA.PIDTuning;

public class Shooter extends SubsystemBase {

    private SparkFlex leftShooter;
    private SparkFlex rightShooter;
    private SparkFlex motorHood;

    private SparkFlexConfig leftShooterConfig;
    private SparkFlexConfig rightShooterConfig;
    private SparkFlexConfig motorHoodConfig;

    private PIDTuning leftShooterPIDtuning;
    private PIDTuning rightShooterPIDtuning;
    private SparkClosedLoopController leftController;
    private SparkClosedLoopController rightController;
    private SparkClosedLoopController motorHoodController;

    private double leftLastRPM = 0.0;
    private boolean leftRecovering = false;
    private double leftBoostEndTime = 0.0;
    private double leftTargetRPM = 0;

    private double rightLastRPM = 0.0;
    private boolean rightRecovering = false;
    private double rightBoostEndTime = 0.0;
    private double rightTargetRPM = 0;

    private double fuelTimeOfFlight = 0.0;

    private static final double BOOST_DURATION_SEC = 0.1;
    private static final double BOOST_OUTPUT = 0.7;

    private int currentCurrent;
    private final double[] distanceTable = { 24.0, 48.0, 72.0, 96.0, 120.0 };
    private final double[] rpmTable = { 2200, 3000, 3600, 4300, 5000 };

    public Shooter() {

        leftShooter = new SparkFlex(Constants.Shooter.SHOOTER_1_CAN_ID, MotorType.kBrushless);
        rightShooter = new SparkFlex(Constants.Shooter.SHOOTER_2_CAN_ID, MotorType.kBrushless);
        motorHood = new SparkFlex(Constants.Shooter.MOTORHOOD_CAN_ID, MotorType.kBrushless);

        leftShooterPIDtuning = new PIDTuning("left_shooter", 0, 0, 0, 0.0);
        rightShooterPIDtuning = new PIDTuning("right_shooter", 0, 0, 0, 0.0);

        leftShooterConfig = new SparkFlexConfig();
        leftShooterConfig.idleMode(IdleMode.kCoast);
        leftShooterConfig.smartCurrentLimit(80);
        leftShooterConfig.inverted(false);
        rightShooterConfig = new SparkFlexConfig();
        rightShooterConfig.idleMode(IdleMode.kCoast);
        rightShooterConfig.smartCurrentLimit(80);
        rightShooterConfig.inverted(false);
        motorHoodConfig = new SparkFlexConfig();
        motorHoodConfig.idleMode(IdleMode.kBrake);
        motorHoodConfig.smartCurrentLimit(40);
        motorHoodConfig.inverted(false);

        leftShooterConfig.closedLoop.pid(
                leftShooterPIDtuning.getPID()[0],
                leftShooterPIDtuning.getPID()[1],
                leftShooterPIDtuning.getPID()[2]).feedForward.kV(0.000157);

        rightShooterConfig.closedLoop.pid(
                rightShooterPIDtuning.getPID()[0],
                rightShooterPIDtuning.getPID()[1],
                rightShooterPIDtuning.getPID()[2]).feedForward.kV(0.000157);

        leftShooter.configure(leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightShooter.configure(rightShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorHood.configure(motorHoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftController = leftShooter.getClosedLoopController();
        rightController = rightShooter.getClosedLoopController();
        motorHoodController = motorHood.getClosedLoopController();
    }

    @Override
    public void periodic() {
        double leftCurrentRPM = leftShooter.getEncoder().getVelocity();
        double leftRpmDelta = leftCurrentRPM - leftLastRPM;

        double rightCurrentRPM = leftShooter.getEncoder().getVelocity();
        double rightRpmDelta = rightCurrentRPM - rightLastRPM;

        Logger.recordOutput("Shooter/leftVelocity", leftCurrentRPM);
        Logger.recordOutput("Shooter/leftTargetRPM", leftTargetRPM);
        Logger.recordOutput("Shooter/leftRPMDelta", leftRpmDelta);

        Logger.recordOutput("Shooter/rightVelocity", rightCurrentRPM);
        Logger.recordOutput("Shooter/rightTargetRPM", rightTargetRPM);
        Logger.recordOutput("Shooter/rightRPMDelta", rightRpmDelta);

        Logger.recordOutput("Shooter/leftAppliedOutput", leftShooter.getAppliedOutput());
        Logger.recordOutput("Shooter/leftVoltage", leftShooter.getBusVoltage() * leftShooter.getAppliedOutput());
        Logger.recordOutput("Shooter/leftCurrent", leftShooter.getOutputCurrent());

        Logger.recordOutput("Shooter/rightAppliedOutput", rightShooter.getAppliedOutput());
        Logger.recordOutput("Shooter/rightVoltage", rightShooter.getBusVoltage() * rightShooter.getAppliedOutput());
        Logger.recordOutput("Shooter/rightCurrent", rightShooter.getOutputCurrent());

        Logger.recordOutput("Shooter/leftRecovering", leftRecovering);
        Logger.recordOutput("Shooter/leftBoostEndTime", leftBoostEndTime);

        Logger.recordOutput("Shooter/rightRecovering", rightRecovering);
        Logger.recordOutput("Shooter/rightBoostEndTime", rightBoostEndTime);

        if (leftRpmDelta < -50.0 && !leftRecovering) {
            leftRecovering = true;
            leftBoostEndTime = Timer.getFPGATimestamp() + BOOST_DURATION_SEC;
        } else if (leftRecovering && Timer.getFPGATimestamp() > leftBoostEndTime) {
            leftRecovering = false;
        }

        if (leftRecovering) {
            leftShooter.set(BOOST_OUTPUT);
        } else {
            leftController.setSetpoint(leftTargetRPM, ControlType.kVelocity);
        }
        if (!leftRecovering && leftTargetRPM == 0) {
            leftShooter.set(0);
        }

        if (rightRpmDelta < -50.0 && !rightRecovering) {
            rightRecovering = true;
            rightBoostEndTime = Timer.getFPGATimestamp() + BOOST_DURATION_SEC;
        } else if (leftRecovering && Timer.getFPGATimestamp() > rightBoostEndTime) {
            rightRecovering = false;
        }

        if (rightRecovering) {
            rightShooter.set(BOOST_OUTPUT);
        } else {
            rightController.setSetpoint(rightTargetRPM, ControlType.kVelocity);
        }
        if (!leftRecovering && rightTargetRPM == 0) {
            rightShooter.set(0);
        }

        leftLastRPM = leftCurrentRPM;
        rightLastRPM = rightCurrentRPM;
    }

    public Command idle() {
        return new RunCommand(() -> {
            leftTargetRPM = 0;
            rightTargetRPM = 0;
            leftRecovering = false;
            rightRecovering = false;
        }, this);
    }

    public Command shoot(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            leftTargetRPM = setPoint.getAsDouble();
            rightTargetRPM = setPoint.getAsDouble();
        }, this);
    }

    private void reconfigure(int current) {
        if (currentCurrent == current) {
            return;
        }
        currentCurrent = current;
        leftShooterConfig.idleMode(IdleMode.kCoast);
        leftShooterConfig.smartCurrentLimit(80);
        leftShooterConfig.inverted(false);

        leftShooterConfig.closedLoop.pid(
                leftShooterPIDtuning.getPID()[0],
                leftShooterPIDtuning.getPID()[1],
                leftShooterPIDtuning.getPID()[2]).feedForward.kV(0.000157);

        leftShooter.configure(leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightShooterConfig.idleMode(IdleMode.kCoast);
        rightShooterConfig.smartCurrentLimit(80);
        rightShooterConfig.inverted(false);

        leftShooterConfig.closedLoop.pid(
                leftShooterPIDtuning.getPID()[0],
                leftShooterPIDtuning.getPID()[1],
                leftShooterPIDtuning.getPID()[2]).feedForward.kV(0.000157);

        rightShooterConfig.closedLoop.pid(
                rightShooterPIDtuning.getPID()[0],
                rightShooterPIDtuning.getPID()[1],
                rightShooterPIDtuning.getPID()[2]).feedForward.kV(0.000157);

        leftShooter.configure(leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightShooter.configure(rightShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private double interpolateRPM(double distanceInches) {
        if (distanceInches <= distanceTable[0])
            return rpmTable[0];
        if (distanceInches >= distanceTable[distanceTable.length - 1])
            return rpmTable[rpmTable.length - 1];
        for (int i = 0; i < distanceTable.length - 1; i++) {
            double d0 = distanceTable[i];
            double d1 = distanceTable[i + 1];
            if (distanceInches >= d0 && distanceInches <= d1) {
                double r0 = rpmTable[i];
                double r1 = rpmTable[i + 1];
                double t = (distanceInches - d0) / (d1 - d0);
                return r0 + t * (r1 - r0);
            }
        }
        return rpmTable[0];

    }

    public Command motorReconfig() {
        return new InstantCommand(() -> {
            reconfigure(80); // we will move this to 120 later
            System.out.println("left_shooter P: " + leftShooterPIDtuning.getPID()[0]);
            System.out.println("left_shooter I: " + leftShooterPIDtuning.getPID()[1]);
            System.out.println("left_shooter D: " + leftShooterPIDtuning.getPID()[2]);
            System.out.println("right_shooter P: " + rightShooterPIDtuning.getPID()[0]);
            System.out.println("right_shooter I: " + rightShooterPIDtuning.getPID()[1]);
            System.out.println("right_shooter D: " + rightShooterPIDtuning.getPID()[2]);
        });
    }

    public Command adjustHood(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            motorHoodController.setSetpoint(leftTargetRPM, ControlType.kPosition);
            motorHoodController.setSetpoint(rightTargetRPM, ControlType.kPosition);
        }, this);
    }

    public double getFuelTimeOfFlight() {
        return fuelTimeOfFlight;
    }
}
