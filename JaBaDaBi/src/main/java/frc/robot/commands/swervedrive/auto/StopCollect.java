package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.NeonLights;

public class StopCollect extends ParallelCommandGroup {
    NeonLights lights;
    Collector collector;

    public StopCollect(NeonLights lights, Collector collector) {
        this.lights = lights;
        this.collector = collector;
        addRequirements(collector);
        addCommands(
            collector.collect(() -> 0.0),
            new InstantCommand(() -> lights.set(new NeonLights.Pattern[]{NeonLights.Pattern.YELLOW}))
        );
    }
}
