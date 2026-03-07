package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.NeonLights;

public class Climb extends SequentialCommandGroup {
    Timer timer;
    NeonLights lights;
    Climber climber;

    public Climb(Climber climber, NeonLights lights) {
        this.climber = climber;
        this.lights = lights;
        addRequirements(climber, lights);
        addCommands(
            new ParallelCommandGroup(
            climber.setClimber(() -> 6.7),
            new InstantCommand(() -> lights.set(new NeonLights.Pattern[]{NeonLights.Pattern.BLUE, NeonLights.Pattern.FAST_FLASH})))
        );
    }
}
