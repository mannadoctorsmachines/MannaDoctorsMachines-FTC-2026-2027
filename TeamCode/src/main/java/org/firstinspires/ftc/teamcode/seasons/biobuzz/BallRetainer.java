package org.firstinspires.ftc.teamcode.seasons.biobuzz;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Rampa de retenção: segura a bola depois do Intake e antes do Shooter, e a
 * libera quando a sequência de disparo pede.
 *
 * API puramente mecânica (block/release): não conhece Shooter, Flower,
 * pontuação nem a sequência de disparo (isso é de um Command da Season).
 * O servo não é exposto.
 */
public class BallRetainer {

    public enum State {
        /** Nenhuma posição foi comandada desde o init. */
        UNKNOWN,
        BLOCKING,
        RELEASED
    }

    private Servo servo;
    private State state = State.UNKNOWN;

    public void init(HardwareMap hardwareMap) {
        state = State.UNKNOWN;
        servo = null;

        // Uma configuração física inválida não deve nem tentar localizar o
        // dispositivo placeholder no HardwareMap, muito menos mover o servo.
        if (!isConfigured()) {
            return;
        }

        servo = hardwareMap.get(Servo.class, BallRetainerConfig.SERVO);
    }

    /** true apenas quando as posições são finitas, válidas e distintas. */
    public boolean isConfigured() {
        double block = BallRetainerConfig.BLOCK_POSITION;
        double release = BallRetainerConfig.RELEASE_POSITION;
        return isFinite(block)
                && isFinite(release)
                && block >= 0.0 && block <= 1.0
                && release >= 0.0 && release <= 1.0
                && block != release
                && BallRetainerConfig.SERVO != null
                && !BallRetainerConfig.SERVO.trim().isEmpty();
    }

    private boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    /** Segura a bola. Não faz nada se o mecanismo não estiver configurado. */
    public void block() {
        if (servo == null || !isConfigured()) {
            return;
        }

        servo.setPosition(BallRetainerConfig.BLOCK_POSITION);
        state = State.BLOCKING;
    }

    /** Libera a bola. Não faz nada se o mecanismo não estiver configurado. */
    public void release() {
        if (servo == null || !isConfigured()) {
            return;
        }

        servo.setPosition(BallRetainerConfig.RELEASE_POSITION);
        state = State.RELEASED;
    }

    public State getState() {
        return state;
    }
}
