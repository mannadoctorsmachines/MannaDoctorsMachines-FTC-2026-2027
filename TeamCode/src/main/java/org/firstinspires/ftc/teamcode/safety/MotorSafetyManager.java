package org.firstinspires.ftc.teamcode.safety;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Monitora a corrente dos motores do robô.
 *
 * IMPORTANTE:
 * Esta classe é somente de diagnóstico.
 *
 * Ela NÃO:
 * - altera potência;
 * - altera velocidade;
 * - para motores;
 * - reduz potência;
 * - interfere nos comandos dos subsistemas.
 *
 * Ela apenas lê:
 * - corrente em ampères;
 * - estado de overcurrent;
 * - tempo em overcurrent.
 */
public class MotorSafetyManager {

    private static class MotorMonitor {

        final String name;
        final DcMotorEx motor;

        long overCurrentStartTime = 0;
        boolean overCurrent = false;

        double currentAmps = 0.0;

        MotorMonitor(String name, DcMotorEx motor) {
            this.name = name;
            this.motor = motor;
        }
    }

    private final Map<String, MotorMonitor> motors = new LinkedHashMap<>();

    private long lastUpdateTime = 0;

    /**
     * Registra um motor para monitoramento.
     *
     * O motor continua sendo controlado normalmente pelo seu subsistema.
     */
    public void registerMotor(String name, DcMotorEx motor) {

        if (!MotorSafetyConfig.ENABLED) {
            return;
        }

        if (motor == null) {
            return;
        }

        if (motors.containsKey(name)) {
            return;
        }

        motors.put(name, new MotorMonitor(name, motor));
    }

    /**
     * Atualiza as informações de todos os motores registrados.
     *
     * Nenhum comando é enviado aos motores neste método.
     */
    public void update() {

        if (!MotorSafetyConfig.ENABLED) {
            return;
        }

        long now = System.currentTimeMillis();

        if (MotorSafetyConfig.UPDATE_INTERVAL_MS > 0
                && now - lastUpdateTime < MotorSafetyConfig.UPDATE_INTERVAL_MS) {
            return;
        }

        lastUpdateTime = now;

        for (MotorMonitor monitor : motors.values()) {

            monitor.currentAmps = monitor.motor.getCurrent(CurrentUnit.AMPS);
            boolean currentlyOverCurrent = monitor.motor.isOverCurrent();

            if (currentlyOverCurrent) {

                if (!monitor.overCurrent) {
                    monitor.overCurrentStartTime = now;
                }

                monitor.overCurrent = true;

            } else {

                monitor.overCurrent = false;
                monitor.overCurrentStartTime = 0;
            }
        }
    }

    /**
     * Retorna a corrente atual de um motor em ampères.
     */
    public double getCurrentAmps(String name) {

        MotorMonitor monitor = motors.get(name);

        if (monitor == null) {
            return 0.0;
        }

        return monitor.currentAmps;
    }

    /**
     * Retorna se o motor está atualmente em overcurrent.
     */
    public boolean isOverCurrent(String name) {

        MotorMonitor monitor = motors.get(name);

        return monitor != null && monitor.overCurrent;
    }

    /**
     * Retorna há quanto tempo o motor está em overcurrent.
     *
     * Retorna 0 quando o motor não está em overcurrent.
     */
    public long getOverCurrentDurationMs(String name) {

        MotorMonitor monitor = motors.get(name);

        if (monitor == null || !monitor.overCurrent) {
            return 0;
        }

        return System.currentTimeMillis() - monitor.overCurrentStartTime;
    }

    /**
     * Retorna a soma das correntes dos motores registrados.
     *
     * ATENÇÃO:
     * Este valor é a soma das leituras individuais dos motores.
     * Não representa necessariamente a corrente exata da bateria.
     */
    public double getTotalCurrentAmps() {

        double total = 0.0;

        for (MotorMonitor monitor : motors.values()) {
            total += monitor.currentAmps;
        }

        return total;
    }

    /**
     * Retorna a quantidade de motores monitorados.
     */
    public int getMotorCount() {
        return motors.size();
    }

    /**
     * Adiciona todas as informações dos motores à telemetria.
     */
    public void addTelemetry(Telemetry telemetry) {

        if (!MotorSafetyConfig.ENABLED) {
            return;
        }

        telemetry.addLine("----- MOTOR SAFETY -----");

        for (MotorMonitor monitor : motors.values()) {

            String status = monitor.overCurrent ? "OVERCURRENT" : "OK";
            telemetry.addData(monitor.name, "%.2f A | %s", monitor.currentAmps, status);

            if (monitor.overCurrent) {
                telemetry.addData(monitor.name + " OverCurrent Time", "%d ms", getOverCurrentDurationMs(monitor.name));
            }
        }

        telemetry.addData("Total Motor Current", "%.2f A", getTotalCurrentAmps());
    }
}