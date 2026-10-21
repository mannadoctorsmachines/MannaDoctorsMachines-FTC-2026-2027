package org.firstinspires.ftc.teamcode.seasons.biobuzz;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * "Guarda-chuva": recebe/posiciona a bola para o disparo e faz um movimento
 * de alavanca para a frente que ajuda a direcionar a bola.
 *
 * API puramente mecânica (repouso / frente): não conhece Shooter, Flower,
 * pontuação nem a sequência de disparo (isso é de um Command da Season).
 * O motor não é exposto.
 *
 * Usa RUN_TO_POSITION; ao iniciar fica com potência 0 (não se move até
 * receber um comando). Sem calibração completa em UmbrellaConfig, nenhuma
 * movimentação é feita.
 */
public class Umbrella {

    private DcMotorEx motor;

    public void init(HardwareMap hardwareMap) {
        motor = null;

        if (!isConfigured()) {
            // Não procura o dispositivo placeholder nem toca no motor enquanto
            // nome, posições e potência não estiverem configurados e válidos.
            return;
        }

        motor = hardwareMap.get(DcMotorEx.class, UmbrellaConfig.MOTOR);
        motor.setDirection(UmbrellaConfig.DIRECTION);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.0);
    }

    /** true se posições e potência já foram calibradas. */
    public boolean isConfigured() {
        double rest = UmbrellaConfig.REST_POSITION_TICKS;
        double forward = UmbrellaConfig.FORWARD_POSITION_TICKS;
        double power = UmbrellaConfig.MOVE_POWER;
        return isFinite(rest)
                && isFinite(forward)
                && isFinite(power)
                && rest >= Integer.MIN_VALUE && rest <= Integer.MAX_VALUE
                && forward >= Integer.MIN_VALUE && forward <= Integer.MAX_VALUE
                && rest != forward
                && power > 0.0 && power <= 1.0
                && UmbrellaConfig.MOTOR != null
                && !UmbrellaConfig.MOTOR.trim().isEmpty()
                && UmbrellaConfig.DIRECTION != null;
    }

    private boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    /** Move para a posição de repouso (recebe a bola). */
    public void moveToRest() {
        moveTo(UmbrellaConfig.REST_POSITION_TICKS);
    }

    /** Move para a frente (alavanca). */
    public void moveToForward() {
        moveTo(UmbrellaConfig.FORWARD_POSITION_TICKS);
    }

    public boolean isBusy() {
        return motor != null && isConfigured() && motor.isBusy();
    }

    /** Para o movimento. Seguro mesmo sem configuração. */
    public void stop() {
        if (motor != null) {
            motor.setPower(0.0);
        }
    }

    private void moveTo(double ticks) {
        if (motor == null || !isConfigured()) {
            return;
        }

        motor.setTargetPosition((int) Math.round(ticks));
        motor.setPower(Math.abs(UmbrellaConfig.MOVE_POWER));
    }
}
