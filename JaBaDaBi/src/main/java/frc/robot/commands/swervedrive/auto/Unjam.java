package frc.robot.commands.swervedrive.auto;

import java.time.Duration;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Collector;

public class Unjam extends SequentialCommandGroup {

    public Unjam(Collector collector, Loader loader, Conveyor conveyor, double duration) {
        addCommands(
                new ParallelDeadlineGroup(
                        new WaitCommand(duration),
                        collector.deploy(() -> 0.34),
                        collector.collect(() -> 1),
                        loader.runLoader(() -> -0.7),
                        conveyor.unloadFuel(() -> true)),
                        collector.idle(),
                        loader.idle(),
                        conveyor.idle());
    }
}
