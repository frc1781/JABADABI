package frc.robot.commands.telecommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Deploy;

public class Collecting {

    public static Command collect(RobotContainer robotContainer) {
        return new RunCommand(() -> {
            robotContainer.getIntake().intakeMotorPower = 1.0;
            robotContainer.getDeploy().collectorTarget = robotContainer.getDeploy().COLLECT_SET_POINT;
            //Logger.recordOutput(getName() + "/currentCommand", "collecting");
        });
    }

}
