package frc.robot.commands.swervedrive.auto;

import java.time.Duration;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Collector;

public class Unjam extends SequentialCommandGroup {
    Collector collector;
    Loader loader;
    Conveyor conveyor;

    public Unjam(Collector collector, Loader loader, Conveyor conveyor, double length) {
        addRequirements(collector, loader, conveyor);
        addCommands(
                collector.deploy(() -> 0.34),
                new ParallelRaceGroup(
                        new WaitCommand(length),
                        collector.intakeSetMotorPower(() -> 1),
                        collector.deploy(() -> 0.33),
                        loader.runLoader(() -> -0.7),
                        conveyor.unloadFuel(() -> true)),
                collector.idle(),
                loader.idle(),
                conveyor.idle());
    }
}
