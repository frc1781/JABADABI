package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

public class Deploy extends Command {
    Collector collector;

    public Deploy(Collector collector) {
        this.collector = collector;
        addRequirements(collector);
    }

    @Override
    public void initialize() {
        System.out.println(getName() + "started");
    }

    @Override
    public void execute() {
        collector.setCollector(() -> 0.321);
    }

    @Override
    public boolean isFinished() {
        return collector.getAbsoluteEncoder().getPosition() < 0.34;
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
