package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Lights;

public class Ascend extends SequentialCommandGroup {
    Timer timer;
    Lights lights;
    Climber climber;

    public Ascend(Climber climber, Lights lights) {
        this.climber = climber;
        this.lights = lights;
        addRequirements(climber);
        addCommands(
            new ParallelCommandGroup( 
            climber.setClimber(() -> 0),
            new InstantCommand(() -> lights.run(Lights.Colors.PURPLE, Lights.Patterns.MARCH)))
        );
    }
}
