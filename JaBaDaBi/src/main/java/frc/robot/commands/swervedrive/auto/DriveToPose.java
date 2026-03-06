package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeonLights;

public class DriveToPose extends Command {

    NeonLights lights;

    public DriveToPose(NeonLights lights) {
        this.lights = lights;
    }

    @Override
    public void initialize() {
        System.out.println(getName() + "started");
    }

    @Override
    public void execute() {
        lights.set(new NeonLights.Pattern[]{NeonLights.Pattern.FIRE_GRADIENT, NeonLights.Pattern.TRAVEL});
    }

    @Override
    public boolean isFinished() {
        return true;
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
