package org.firstinspires.ftc.teamcode.safety;

/**
 * Configuração do monitoramento de segurança dos motores.
 *
 * Esta classe NÃO controla os motores.
 * Ela apenas define como as informações serão apresentadas.
 */
public class MotorSafetyConfig {

    /**
     * Ativa ou desativa o monitoramento de corrente.
     */
    public static final boolean ENABLED = true;

    /**
     * Intervalo mínimo entre leituras de corrente.
     *
     * 0 = lê em todas as atualizações.
     */
    public static final long UPDATE_INTERVAL_MS = 0;

    private MotorSafetyConfig() {
        // Classe somente de configuração.
    }
}