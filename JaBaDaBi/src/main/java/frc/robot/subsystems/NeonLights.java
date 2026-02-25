package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NeonLights extends SubsystemBase { // unreasonably proud of this name
    interface PatternFunction {
        Color apply(Color color, int i);
    }

    private static class Color {
        public int r;
        public int g;
        public int b;

        public Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }

        public Color(int h) {
            if(h < 60) {
                r = 255;
                g = Math.round((h/60f) * 255f);
                b = 0;
            } else if(h < 120) {
                r = Math.round(((120 - h)/60f) * 255f);
                g = 255;
                b = 0;
            } else if(h < 180) {
                r = 0;
                g = 255;
                b = Math.round(((h - 120)/60f) * 255f);
            } else if(h < 240) {
                r = 0;
                g = Math.round(((240 - h)/60f) * 255f);
                b = 255;
            } else if(h < 300) {
                r = Math.round(((h - 240)/60f) * 255f);
                g = 0;
                b = 255;
            } else {
                r = 255;
                g = 0;
                b = Math.round(((360 - h)/60f) * 255f);
            }
        }

        public Color() {
            r = 0;
            g = 0;
            b = 0;
        }
    }

    private AddressableLED controller = null;
    private AddressableLEDBuffer buffer = null;

    static private double[] noise;

    static {
        noise = new double[frc.robot.Constants.LED_LENGTH];
        for(int i = 0; i < frc.robot.Constants.LED_LENGTH; i++) {
            noise[i] = Math.random();
        }
    }

    public NeonLights() {
        controller = new AddressableLED(1);
        buffer = new AddressableLEDBuffer(frc.robot.Constants.LED_LENGTH + 1);
        controller.setLength(buffer.getLength());
        controller.setData(buffer);
        controller.start();
    }

    public void periodic() {
        int[] rgbData = new int[buffer.getLength() * 3];
        for (int i = 0; i < buffer.getLength(); i++) {
            rgbData[i * 3 + 0] = buffer.getRed(i);
            rgbData[i * 3 + 1] = buffer.getGreen(i);
            rgbData[i * 3 + 2] = buffer.getBlue(i);
        }
        Logger.recordOutput("LEDs/Strip", rgbData);
    }

    public void run(Pattern[] patterns) {
        for(int i = 0; i < frc.robot.Constants.LED_LENGTH; i++) {
            Color color = new Color();
            for(int p = 0; p < patterns.length; p++) {
                color = patterns[i].function.apply(color, i); // i feel so fucking cool right now
            }
            buffer.setRGB(i, color.r, color.g, color.b);
        }
        controller.setData(buffer);
    }

    public Command set(Pattern[] patterns) {
        return new RunCommand(() -> {
            run(patterns);
        }, this);
    }

    public enum Pattern {
        OFF((Color color, int i) -> { return new Color(); }),
        RED((Color color, int i) -> { return new Color(255, 0, 0); }),
        ORANGE((Color color, int i) -> { return new Color(255, 150, 0); }),
        YELLOW((Color color, int i) -> { return new Color(255, 255, 0); }),
        GREEN((Color color, int i) -> { return new Color(0, 255, 0); }),
        BLUE((Color color, int i) -> { return new Color(0, 0, 255); }),
        PURPLE((Color color, int i) -> { return new Color(255, 0, 255); }),
        WHITE((Color color, int i) -> { return new Color(254, 254, 254); }),
        BLINK((Color color, int i) -> { return blink(color, i, 400); }),
        FAST_BLINK((Color color, int i) -> { return blink(color, i, 200); }),
        FLASH((Color color, int i) -> { return flash(color, i, 200); }),
        FAST_FLASH((Color color, int i) -> { return flash(color, i, 400); }),
        MARCH((Color color, int i) -> { return march(color, i, 200, 3); }),
        TRAVEL((Color color, int i) -> { return march(color, i, 80, frc.robot.Constants.LED_LENGTH - 1); }),
        RAINBOW((Color color, int i) -> { return rainbow(i, 1); }),
        RAINBOW_SOLID((Color color, int i) -> { return rainbow(i, 0); });

        private PatternFunction function;
        private Pattern(PatternFunction function) {
            this.function = function;
        }

        private static Color blink(Color color, int i, double delay) {
            if(System.currentTimeMillis() % (delay * 2) <= delay) {
                return color;
            } else {
                return new Color();
            }
        }

        private static Color flash(Color color, int i, double delay) {
            if((System.currentTimeMillis() % (delay * 2) <= delay) == (i % 2 <= 1)) {
                return color;
            } else {
                return new Color();
            }
        }

        private static Color march(Color color, int i, double delay, int ammount) {
            if(i % ammount == Math.floor((System.currentTimeMillis() % (delay * ammount))/delay)) {
                return color;
            } else {
                return new Color();
            }
        }

        private static Color rainbow(int i, int speed) {
            int hue = ((i * 4) + ((int)System.currentTimeMillis()) * speed)% 255;
            return new Color(hue);
        }
    }
}
