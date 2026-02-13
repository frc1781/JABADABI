package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;

public class Shoot extends SequentialCommandGroup {

    Loader loader;
    Conveyor conveyor;

    public Shoot(Loader loader, Conveyor conveyor) {

        this.loader = loader;
        this.conveyor = conveyor;

        addRequirements(loader, conveyor);
        loader.runLoader(() -> 1);
        conveyor.loadFuel(() -> true);
    }

}
