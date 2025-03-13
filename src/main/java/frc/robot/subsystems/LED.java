package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class LED extends SubsystemBase {
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
        commands.put(LEDState.RAINBOW, new ChromaLED(this, (double i) -> Color.fromHSV((int)Math.floor(i * 180), 255, 255)).repeatedly());
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
        startLED();
    }

    public void bindButton(JoystickButton button, LEDState state) {
        button.onTrue(new InstantCommand(() -> activateState(state), this));
        button.onFalse(new InstantCommand(() -> deactivateState(state), this));
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

    public static class BlinkLED extends SequentialCommandGroup {
        private BlinkLED(LED led){
            super(
                new InstantCommand(() -> led.stopLED()),
                new WaitCommand(0.5d),
                new InstantCommand(() -> led.startLED()),
                new WaitCommand(0.5d)
            );
        }
    }

    private static class ChromaLED extends Command {
        private LED led;
        private LEDColorSupplier supplier;

        private ChromaLED(LED led, LEDColorSupplier supplier) {
            this.led = led;
            this.supplier = supplier;
            addRequirements(led);
        }

        @Override
        public void execute() {
            int len = led.buffer.getLength();
            int offset = (int)Math.floor((System.currentTimeMillis()/10) % len);
            for (int i = 0; i < len; i++)
                led.buffer.setLED((i+offset) % len, supplier.get((double)i/len));
            led.strip.setData(led.buffer);
        }

        public static interface LEDColorSupplier {
            public Color get(double progress);
        }
    }
}