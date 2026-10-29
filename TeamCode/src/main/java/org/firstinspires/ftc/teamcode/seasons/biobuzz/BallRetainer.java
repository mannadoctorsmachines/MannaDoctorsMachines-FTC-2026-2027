package org.firstinspires.ftc.teamcode.seasons.biobuzz;

import com.qualcomm.robotcore.hardware.CRServo;
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
    private CRServo continuousServo;
    private State state = State.UNKNOWN;
    private long movementStartTime = 0;
    private boolean moving = false;

    public void init(HardwareMap hardwareMap) {
        state = State.UNKNOWN;
        servo = null;
        continuousServo = null;
        moving = false;

        // Uma configuração física inválida não deve nem tentar localizar o
        // dispositivo placeholder no HardwareMap, muito menos mover o servo.
        if (!isConfigured()) {
            return;
        }

        if (BallRetainerConfig.CONTINUOUS) {
            continuousServo = hardwareMap.get(
                    CRServo.class,
                    BallRetainerConfig.SERVO
            );
            continuousServo.setPower(0.0);
        } else {
            servo = hardwareMap.get(
                    Servo.class,
                    BallRetainerConfig.SERVO
            );
        }
    }

    /** true apenas quando as posições são finitas, válidas e distintas. */
    public boolean isConfigured() {
        if(BallRetainerConfig.SERVO == null
                || BallRetainerConfig.SERVO.trim().isEmpty()){
            return false;
        }

        if(BallRetainerConfig.CONTINUOUS){
            return isFinite(BallRetainerConfig.BLOCK_POWER)
                    && isFinite(BallRetainerConfig.RELEASE_POWER)
                    && BallRetainerConfig.BLOCK_POWER >= -1.0
                    && BallRetainerConfig.BLOCK_POWER <= 1.0
                    && BallRetainerConfig.RELEASE_POWER >= -1.0
                    && BallRetainerConfig.RELEASE_POWER <= 1.0
                    && BallRetainerConfig.MOVEMENT_TIME_MS > 0;
        }

        double block = BallRetainerConfig.BLOCK_POSITION;
        double release = BallRetainerConfig.RELEASE_POSITION;

        return isFinite(block)
                && isFinite(release)
                && block >= 0.0 && block <= 1.0
                && release >= 0.0 && release <= 1.0
                && block != release;
    }

    private boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    /** Segura a bola. Não faz nada se o mecanismo não estiver configurado. */
    public void block() {
        if (servo == null || !isConfigured()) {
            return;
        }

        if (BallRetainerConfig.CONTINUOUS) {
            continuousServo.setPower(BallRetainerConfig.BLOCK_POWER);
            movementStartTime = System.currentTimeMillis();
            moving = true;
        } else {
            servo.setPosition(BallRetainerConfig.BLOCK_POSITION);
        }

        state = State.BLOCKING;
    }

    /** Libera a bola. Não faz nada se o mecanismo não estiver configurado. */
    public void release() {
        if (!isConfigured()) {
            return;
        }

        if (BallRetainerConfig.CONTINUOUS) {
            continuousServo.setPower(BallRetainerConfig.RELEASE_POWER);
            movementStartTime = System.currentTimeMillis();
            moving = true;
        } else {
            servo.setPosition(BallRetainerConfig.RELEASE_POSITION);
        }

        state = State.RELEASED;
    }

    public void update() {
        if (!BallRetainerConfig.CONTINUOUS || !moving) {
            return;
        }

        long elapsed = System.currentTimeMillis() - movementStartTime;

        if (elapsed >= BallRetainerConfig.MOVEMENT_TIME_MS) {
            continuousServo.setPower(0.0);
            moving = false;
        }
    }

    public State getState() {
        return state;
    }
}
