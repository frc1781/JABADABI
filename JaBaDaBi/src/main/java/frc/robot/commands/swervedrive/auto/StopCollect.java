package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.Colors;
import frc.robot.subsystems.Lights.Patterns;

public class StopCollect extends Command {
    Lights lights;
    Collector collector;

    public StopCollect(Lights lights, Collector collector) {
        this.lights = lights;
        this.collector = collector;
        addRequirements(collector);
    }

    @Override
    public void initialize() {
        System.out.println(getName() + "started");
    }

    @Override
    public void execute() {
        collector.collect(() -> 0.0);
        lights.run(Colors.YELLOW, Patterns.MARCH);  //change this
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println(getName() + "interrupted");
        } else {
            System.out.println(getName() + "ended");
        }
    }
}
