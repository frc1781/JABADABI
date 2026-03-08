package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Collector;

public class Deploy extends SequentialCommandGroup {
    Collector collector;

    public Deploy(Collector collector) {
        this.collector = collector;
        addRequirements(collector);
        addCommands(
            collector.deploy(() -> 0.34)
        );
    }
}
