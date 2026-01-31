package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class EEUtil {
    public static Rotation2d normalizeAngle(Rotation2d angle) {
        double radians = angle.getRadians();

        radians %= 2 * Math.PI;
        if(radians<0) {
            radians += 2 * Math.PI;
        }

        return Rotation2d.fromRadians(radians);
    }

    public static double clamp(double min, double max, double input) {
        return Math.min(max, Math.max(input, min));
    }

    public static double angleDiffDegrees(double angle1, double angle2) {
        if (Math.abs(angle2 - angle1) > 180) {
            return 360 - Math.abs(angle2 - angle1);
        }
        return Math.abs(angle2 - angle1);
    }
}
