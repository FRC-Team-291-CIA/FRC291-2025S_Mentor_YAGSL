package frc.robot.subsystems.candle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import frc.robot.Constants.CandleConstants;

public class CandleSubsystem extends SubsystemBase {
    private final int LEDS_PER_ANIMATION = 144;
    private final CANdle m_candle = new CANdle(CandleConstants.CANdleID, "rio");
    private int m_candleChannel = 0;
    private boolean m_last5V = false;
    private boolean m_animDirection = false;

    private Animation m_toAnimate;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        Empty
    }

    private AnimationTypes m_currentAnimation;

    public CandleSubsystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);

        m_candle.setLEDs(255, 255, 255, 0, 0, 1);
        m_candle.setLEDs(255, 255, 0, 0, 1, 1);
        m_candle.setLEDs(255, 0, 255, 0, 2, 1);
        m_candle.setLEDs(255, 0, 0, 0, 3, 1);
        m_candle.setLEDs(0, 255, 255, 0, 4, 1);
        m_candle.setLEDs(0, 255, 0, 0, 5, 1);
        m_candle.setLEDs(0, 0, 0, 0, 6, 1);
        m_candle.setLEDs(0, 0, 255, 0, 7, 1);

        this.setWantedState(AnimationTypes.Fire);
    }

    public void toggle5VOverride() {
        System.out.println("State is: " + m_last5V);
        m_candle.configV5Enabled(m_last5V);
        m_last5V = !m_last5V;
    }

    public void toggleAnimDirection() {
        m_animDirection = !m_animDirection;
    }

    public int getMaximumAnimationCount() {
        return m_candle.getMaxSimultaneousAnimationCount();
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() {
        return m_candle.getBusVoltage();
    }

    public double get5V() {
        return m_candle.get5VRailVoltage();
    }

    public double getCurrent() {
        return m_candle.getCurrent();
    }

    public double getTemperature() {
        return m_candle.getTemperature();
    }

    public void configBrightness(double percent) {
        m_candle.configBrightnessScalar(percent, 0);
    }

    public void configLos(boolean disableWhenLos) {
        m_candle.configLOSBehavior(disableWhenLos, 0);
    }

    public void configLedType(LEDStripType type) {
        m_candle.configLEDType(type, 0);
    }

    public void configStatusLedBehavior(boolean offWhenActive) {
        m_candle.configStatusLedState(offWhenActive, 0);
    }

    public void setWantedState(AnimationTypes toChange) {
        m_currentAnimation = toChange;

        switch (toChange) {
            default:
            case ColorFlow:
                m_candleChannel = 0;
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LEDS_PER_ANIMATION, Direction.Forward,
                        8);
                break;
            case Fire:
                m_candleChannel = 0;
                m_toAnimate = new FireAnimation(1, 0.7, LEDS_PER_ANIMATION, 0.8, 0, m_animDirection,
                        8);
                break;
            case Larson:
                m_candleChannel = 0;
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 0.1, LEDS_PER_ANIMATION, BounceMode.Front, 3,
                        8);
                break;
            case Rainbow:
                m_candleChannel = 0;
                m_toAnimate = new RainbowAnimation(1, 0.7, LEDS_PER_ANIMATION, m_animDirection,
                        8);
                break;
            case RgbFade:
                m_candleChannel = 0;
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LEDS_PER_ANIMATION,
                        8);
                break;
            case SingleFade:
                m_candleChannel = 0;
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LEDS_PER_ANIMATION,
                        8);
                break;
            case Strobe:
                m_candleChannel = 0;
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 0.01, LEDS_PER_ANIMATION,
                        8);
                break;
            case Twinkle:
                m_candleChannel = 0;
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LEDS_PER_ANIMATION, TwinklePercent.Percent42,
                        8);
                break;
            case TwinkleOff:
                m_candleChannel = 0;
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.2, LEDS_PER_ANIMATION,
                        TwinkleOffPercent.Percent76, LEDS_PER_ANIMATION + 8);
                break;
            case Empty:
                m_candleChannel = 0;
                m_toAnimate = new RainbowAnimation(1, 0.7, LEDS_PER_ANIMATION, m_animDirection,
                        LEDS_PER_ANIMATION + 8);
                break;
        }
        System.out.println("Changed to " + m_currentAnimation.toString());

        m_candle.animate(m_toAnimate, m_candleChannel);
    }

}