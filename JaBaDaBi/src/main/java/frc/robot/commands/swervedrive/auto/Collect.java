package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.NeonLights;

public class Collect extends SequentialCommandGroup {
    NeonLights lights;
    Collector collector;

    public Collect(NeonLights lights, Collector collector) {
        this.lights = lights;
        this.collector = collector;
        addRequirements(collector);
        addCommands(
            new ParallelCommandGroup(
                collector.deploy(() -> 0.34), 
                collector.intakeSetMotorPower(() -> 1)
            )
            // new InstantCommand(() -> lights.set(new NeonLights.Pattern[]{NeonLights.Pattern.YELLOW, NeonLights.Pattern.MARCH})))
        );
  
    }
}
