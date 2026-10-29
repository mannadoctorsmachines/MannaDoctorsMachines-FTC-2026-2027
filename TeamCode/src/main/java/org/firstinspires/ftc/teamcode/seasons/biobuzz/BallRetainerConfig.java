package org.firstinspires.ftc.teamcode.seasons.biobuzz;

/**
 * Configuração da rampa de retenção da bola (servo).
 *
 * TODOS os valores físicos abaixo são PENDENTES DE CALIBRAÇÃO / TESTE FÍSICO.
 * Enquanto uma posição for Double.NaN, o mecanismo é considerado "não
 * configurado" e NÃO move o servo.
 */
public class BallRetainerConfig {

    // Liga o mecanismo na composição (BiobuzzRobot). Ative só depois de
    // preencher o nome do servo no Driver Hub e as duas posições.
    public static final boolean ENABLED = true;
    public static final boolean CONTINUOUS = false;

    // PENDENTE: nome do servo no Driver Hub (confirmar/ajustar).
    public static final String SERVO = "retainerServo";

    // PENDENTE: posições do servo (0.0 a 1.0) medidas no robô.
    // Servo posicional
    public static final double BLOCK_POSITION = 1.0; //Double.NaN;
    public static final double RELEASE_POSITION = 0.0; //Double.NaN;

    // Servo continuo
    public static final double BLOCK_POWER = 0.0; //Double.NaN;
    public static final double RELEASE_POWER = 0; //Double.NaN;
    // tempo do movimento do servo contínuo
    public static final long MOVEMENT_TIME_MS = 300; //Double.NaN;
}
