package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.FrequencyUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.datalog.RawLogEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LED extends SubsystemBase {

    private LEDPattern pattern;
    public enum LEDState {
        WHITE, BLACK, 
        RED, GREEN, BLUE, 
        TURQUOISE, PURPLE, YELLOW,
        RAINBOW,
    }

    private Map<LEDState, Command> commands = new HashMap<LEDState, Command>();
    {
        commands.put(LEDState.WHITE, new InstantCommand(() -> setLED(75, 75, 75), this));
        commands.put(LEDState.BLACK, new InstantCommand(() -> setLED(0, 0, 0), this));
        commands.put(LEDState.RED, new InstantCommand(() -> setLED(255, 0, 0)));
        commands.put(LEDState.GREEN, new InstantCommand(() -> setLED(0, 255, 0)));
        commands.put(LEDState.BLUE, new InstantCommand(() -> setLED(0, 0, 255)));
        commands.put(LEDState.TURQUOISE, new InstantCommand(() -> setLED(50, 210, 200)));
        commands.put(LEDState.PURPLE, new InstantCommand(() -> setLED(70, 0, 100), this));
        commands.put(LEDState.YELLOW, new InstantCommand(() -> setLED(150, 75, 0), this));
        commands.put(LEDState.RAINBOW, new ChromaLED((double i) -> Color.fromHSV((int)Math.floor(i * 180), 255, 255)).repeatedly());
    };
    private AddressableLED strip;
    private AddressableLEDBuffer buffer;
    private LEDState state;
    private ArrayList<LEDState> activeStateList;

    public LED() { 
        strip = new AddressableLED(3);
        buffer = new AddressableLEDBuffer(170);
        strip.setLength(buffer.getLength());
        activeStateList = new ArrayList<LEDState>();
        state = LEDState.BLACK;
        light(Color.kBlack);
        startLED();
    }

    public void bindButton(Trigger button, LEDState state) {
        button.onTrue(new InstantCommand(() -> activateState(state), this));
        //button.onFalse(new InstantCommand(() -> deactivateState(state), this));
    }

    public void activateState(LEDState state) {
        if (activeStateList.contains(state)) return;
        activeStateList.add(state);
        setState();
    }

    public void deactivateState(LEDState state) {
        if (!activeStateList.contains(state)) return;
        activeStateList.remove(activeStateList.indexOf(state));
        setState();
    }

    private void setState() {
        getStateCommand().cancel();
        int size = activeStateList.size();
        state = size > 0 ? activeStateList.get(size - 1) : LEDState.BLACK;
        startLED();
    }

    private Command getStateCommand() {
        return commands.get(state);
    }

    private void startLED() {
        strip.start();
        getStateCommand().schedule();
    }

    private void stopLED() {
        getStateCommand().cancel();
        setLED(0, 0, 0);
        strip.stop();
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

    public void blink(Color color){
        pattern = LEDPattern.solid(color).blink(Seconds.of(0.5), Seconds.of(0.5));
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
        private int prevr, prevg, prevb;
        private int r, g, b;
        private BooleanSupplier finisher;
        private boolean timeout;
        private Timer timer;
        public BlinkLED(int r, int g, int b, BooleanSupplier finisher){
            this.r = r;
            this.g = g;
            this.b = b;
            this.finisher = finisher;
            timer = new Timer();
            addRequirements(LED.this);
            //LED.this.setLED(r, g, b);
            // addCommands(
            //     new InstantCommand(() -> LED.this.stopLED()),
            //     new WaitCommand(0.5d),
            //     new InstantCommand(() -> LED.this.startLED()),
            //     new WaitCommand(0.5d),
            //     new InstantCommand(() -> LED.this.stopLED()),
            //     new WaitCommand(0.5d),
            //     new InstantCommand(() -> LED.this.startLED()),
            //     new InstantCommand(() -> LED.this.setLED(prevr, prevg, prevb))
            // );
        }

        public BlinkLED(int r, int g, int b, boolean timeout){
            this(r, g, b, () -> false);
            this.timeout = timeout;
        }
        @Override
        public void initialize(){
            prevr = LED.this.buffer.getRed(0);
            prevg = LED.this.buffer.getGreen(0);
            prevb = LED.this.buffer.getBlue(0);
            timer.reset();
        }

        public void execute(){
            if(Math.floor(timer.get()*2) % 2 == 0){
                LED.this.setLED(r, g, b);
            }
            else{
                LED.this.setLED(0, 0, 0);
            }
        }
    
        @Override
        public boolean isFinished(){
            return finisher.getAsBoolean() || (timeout && timer.get() > 2);
        }

        @Override
        public void end(boolean interrupted){
            LED.this.setLED(prevr, prevg, prevb);
            LED.this.startLED();
        }
    }

    public class ChromaLED extends Command {
        private LEDColorSupplier supplier;

        public ChromaLED(LEDColorSupplier supplier) {
            this.supplier = supplier;
            addRequirements(LED.this);
        }

        @Override
        public void execute() {
            int len = LED.this.buffer.getLength();
            int offset = (int)Math.floor((System.currentTimeMillis()/10) % len);
            for (int i = 0; i < len; i++)
                LED.this.buffer.setLED((i+offset) % len, supplier.get((double)i/len));
            LED.this.strip.setData(LED.this.buffer);
        }

        public static interface LEDColorSupplier {
            public Color get(double progress);
        }
    }
}