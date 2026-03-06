package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.Colors;
import frc.robot.subsystems.Lights.Patterns;

public class Collect extends SequentialCommandGroup {
    Lights lights;
    Collector collector;

    public Collect(Lights lights, Collector collector) {
        this.lights = lights;
        this.collector = collector;
        addRequirements(collector);
        addCommands(
            new ParallelCommandGroup(
            collector.deploy(() -> 0.34),
            collector.collect(() -> 80),
            new InstantCommand(() -> lights.run(Colors.YELLOW, Patterns.MARCH)))
        );
  
    }
}
