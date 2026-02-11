import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.TimedRobot;

public class Pigeon2 extends TimedRobot {
    Pigeon2 _pigeon = new Pigeon2(0, "rio");
    int _loopCount = 0;

    public void teleopPeriodic() {
        if(_loopCount++ > 10)
        {
            _loopCount = 0;
            double yaw = _pigeon.getYaw();
            System.out.println("Pigeon Yaw is: " + yaw);
        }
    }