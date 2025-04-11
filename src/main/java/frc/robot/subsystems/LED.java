package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private LEDPattern pattern;
    private AddressableLED strip;
    private AddressableLEDBuffer buffer;

    public LED() { 
        strip = new AddressableLED(3);
        buffer = new AddressableLEDBuffer(170);
        strip.setLength(buffer.getLength());
    }

    private void setLED(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
        strip.setData(buffer);
    }

    public void light(Color color){
        pattern = LEDPattern.solid(color);
    }

    /** default blink time of 0.5 seconds */
    public void blink(Color color){
        blink(color, 0.5);
    }

    public void blink(Color color, double blinkTime){
        pattern = LEDPattern.solid(color).blink(Seconds.of(blinkTime), Seconds.of(blinkTime));
    }

    public void rainbow(){
        pattern = LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Hertz.of(1));
    }

    @Override
    public void periodic(){
        pattern.applyTo(buffer);
        strip.setData(buffer);
    }

    public class LightLED extends Command{
        private int r, g, b;

        public LightLED(int r, int g, int b){
            this.r = r;
            this.g = g;
            this.b = b;
            addRequirements(LED.this);
        }

        @Override
        public void initialize(){
            LED.this.setLED(r, g, b);
        }
    }

    public class BlinkLED extends Command {
        private Color blinkColor;
        private boolean rainbow;
        private Timer timer;
        private double blinkTime, onTime;
        /** @param blinkColor the color that will blink
         * @param onTime the time that the LED will blink for
         * @param blinkTime the amount of time the LED will spend in each on and off state
         * @param rainbow if true the LED will do a rainbow pattern when the time of onTime is reached
         */
        public BlinkLED(Color blinkColor, double onTime, double blinkTime, boolean rainbow){
            this.blinkColor = blinkColor;
            this.onTime = onTime;
            this.rainbow = rainbow;
            timer = new Timer();
            this.blinkTime = blinkTime;
            addRequirements(LED.this);
        }
        @Override
        public void initialize(){
            LED.this.blink(blinkColor, blinkTime);
            timer.restart();
        }

        @Override
        public boolean isFinished(){
            return timer.get() > onTime;
        }

        @Override
        public void end(boolean interrupted){
            if(rainbow){
                LED.this.rainbow();
            }
            else{
                LED.this.light(blinkColor);
            }
        }
    }
}