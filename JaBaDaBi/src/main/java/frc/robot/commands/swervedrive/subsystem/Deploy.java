package frc.robot.commands.swervedrive.subsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Collector;

public class Deploy extends SequentialCommandGroup {
    Collector collector;
    Loader loader;

    public Deploy(Collector collector, Loader loader) {
        this.collector = collector;
        this.loader = loader;
        addRequirements(collector, loader);
        addCommands(
            new ParallelCommandGroup(
            collector.deploy(() -> 0.34),
            loader.runLoader(() -> -0.8))
        );
    }
}
