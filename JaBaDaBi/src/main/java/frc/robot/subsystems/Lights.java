package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Lights extends SubsystemBase
{
    private final int LED_LENGTH = 150;
    
    public enum Colors
    {
        RED,
        ORANGE,
        YELLOW,
        GREEN,
        BLUE,
        PURPLE,
        WHITE
    }
    public enum Patterns 
    {
        SOLID,
        BLINK,
        FAST_BLINK,
        FLASH,
        FAST_FLASH,
        MARCH,
        TRAVEL
    }
    public enum Special 
    {
        OFF,
        RAINBOW
    }

    private AddressableLED controller = null;
    private AddressableLEDBuffer buffer = null;

    private Timer timer;

    public Lights()
    {
        controller = new AddressableLED(1);
        buffer = new AddressableLEDBuffer(LED_LENGTH + 1);
        controller.setLength(buffer.getLength());
        controller.setData(buffer);
        controller.start();
        
        timer = new Timer();
        timer.reset();
        timer.start();
    }

    @Override
    public void periodic() {
        int[] rgbData = new int[buffer.getLength() * 3];
        for (int i = 0; i < buffer.getLength(); i++) {
            rgbData[i * 3 + 0] = buffer.getRed(i);
            rgbData[i * 3 + 1] = buffer.getGreen(i);
            rgbData[i * 3 + 2] = buffer.getBlue(i);
        }

        Logger.recordOutput("LEDs/Strip", rgbData);
    }

    private Color solid(int i, Color color)
    {
        return new Color(color);
    }

    private Color blink(int i, Color color, double delay)
    {
        if(timer.get() > delay)
        {
            if(timer.get() > delay * 2)
            {
                timer.reset();
            }
            return new Color(color);
        }
        else
        {
            return new Color();
        }
    }

    private Color flash(int i, Color color, double delay)
    {
        boolean toggle = timer.get() > delay;
        if(timer.get() > delay * 2)
        {
            timer.reset();
        }
        if((i % 2 == 0) != toggle)
        {
            return new Color(color);
        }
        else
        {
            return new Color();
        }
    }

    private Color march(int i, Color color, double delay, int ammount)
    {
        if(timer.get() > delay * ammount)
        {
            timer.reset();
        }
        if(i % ammount == Math.floor(timer.get()/delay))
        {
            return new Color(color);
        }
        else
        {
            return new Color();
        }
    }

    private Color rainbow(int i)
    {
        if(timer.get() > 1)
        {
            timer.reset();
        }
        int hue = Math.round((float) timer.get() * 360);
        hue = (hue + i * 4) % 360;
        return new Color(hue);
    }

    private Color patternLookup(Patterns pattern, int i, Color color)
    {
        switch(pattern)
        {
            case SOLID:
                return solid(i, color);
            case BLINK:
                return blink(i, color, 0.4);
            case FAST_BLINK:
                return blink(i, color, 0.2);
            case FLASH:
                return flash(i, color, 0.4);
            case FAST_FLASH:
                return flash(i, color, 0.2);
            case MARCH:
                return march(i, color, 0.2, 3);
            case TRAVEL:
                return march(i, color, 0.08, LED_LENGTH - 1);
            default:
                return new Color(0, 0, 0);
        }
    }

    private Color specialLookup(Special special, int i)
    {
        switch(special)
        {
            case RAINBOW:
                return rainbow(i);
            default: case OFF:
                return new Color(0, 0, 0);
        }
    }

    private Color colorLookup(Colors colorName)
    {
        switch(colorName)
        {
            case RED:
                return new Color(255, 0, 0);
            case ORANGE:
                return new Color(255, 150, 0);
            case YELLOW:
                return new Color(255, 255, 0);
            case GREEN:
                return new Color(0, 255, 0);
            case BLUE:
                return new Color(0, 0, 255);
            case PURPLE:
                return new Color(255, 0, 255);
            case WHITE:
                return new Color(254, 254, 254);
            default:
                return new Color(0, 0, 0);
        }
    }

    public void run(Colors colorName, Patterns pattern)
    {
        Color startColor = colorLookup(colorName);
        for(int i = 0; i < LED_LENGTH; i++)
        {
            Color color = patternLookup(pattern, i, startColor);
            buffer.setRGB(i, color.r, color.g, color.b);
        }
        controller.setData(buffer);
    }

    public void runSpecial(Special special)
    {
        for(int i = 0; i < LED_LENGTH; i++)
        {
            Color color = specialLookup(special, i);
            buffer.setRGB(i, color.r, color.g, color.b);
        }
        controller.setData(buffer);
    }

    private class Color 
    {
        public int r;
        public int g;
        public int b;

        public Color(int r, int g, int b)
        {
            this.r = r;
            this.g = g;
            this.b = b;
        }

        public Color(int h)
        {
            if(h < 60)
            {
                r = 255;
                g = Math.round((h/60f) * 255f);
                b = 0;
            }
            else if(h < 120)
            {
                r = Math.round(((120 - h)/60f) * 255f);
                g = 255;
                b = 0;
            }
            else if(h < 180)
            {
                r = 0;
                g = 255;
                b = Math.round(((h - 120)/60f) * 255f);
            }
            else if(h < 240)
            {
                r = 0;
                g = Math.round(((240 - h)/60f) * 255f);
                b = 255;
            }
            else if(h < 300)
            {
                r = Math.round(((h - 240)/60f) * 255f);
                g = 0;
                b = 255;
            }
            else
            {
                r = 255;
                g = 0;
                b = Math.round(((360 - h)/60f) * 255f);
            }
        }

        public Color()
        {
            r = 0;
            g = 0;
            b = 0;
        }

        public Color(Color color)
        {
            r = color.r;
            g = color.g;
            b = color.b;
        }
    }

    public Command set(Colors color, Patterns pattern)
    {
        return new RunCommand(() -> {run(color, pattern);}, this);
    }

    public Command set(Special special)
    {
        return new RunCommand(() -> {runSpecial(special);}, this);
    }
}
