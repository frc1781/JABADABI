package frc.robot.commands.swervedrive.subsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class Collect extends SequentialCommandGroup {
    Collector collector;
    Loader loader;
    Conveyor conveyor;

    public Collect(Collector collector, Loader loader, Conveyor conveyor) {
        this.collector = collector;
        this.loader = loader;
        this.conveyor = conveyor;
        addRequirements(collector, loader, conveyor);
        addCommands(
            new ParallelCommandGroup(
                collector.collect(), 
                loader.runLoader(() -> -0.8),
                conveyor.loadFuel(() -> 1)
            )
        );
    }
}
