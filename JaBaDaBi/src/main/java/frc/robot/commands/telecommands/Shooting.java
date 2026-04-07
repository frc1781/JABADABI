package frc.robot.commands.telecommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class Shooting {

    public static Command shotTesting(RobotContainer robotContainer) {
        return new RunCommand(() -> {
            robotContainer.getShooter().leftReqRPS = 55;
            robotContainer.getShooter().rightReqRPS = 55;
        });
    }
}
