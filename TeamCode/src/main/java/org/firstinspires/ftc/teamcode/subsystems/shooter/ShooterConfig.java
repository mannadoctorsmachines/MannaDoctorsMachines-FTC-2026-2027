package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Configuração do Shooter. Os RPMs representam a velocidade do rolo
 * (lado mecânico de saída), não necessariamente o eixo do motor.
 *
 * Os dois motores paralelos compartilham a mesma configuração.
 * O motor de outtake possui configuração e conversão independentes.
 */
public class ShooterConfig {

    // Hardware Map - Shooter
    public static final String SHOOTER_LEFT = "shooterMotor1";
    public static final String SHOOTER_RIGHT = "shooterMotor2";
    public static final String SHOOTER_OUTTAKE = "shooterMotor3";
    // Confirme o sentido no teste de bancada conforme a montagem física.
    public static final DcMotorSimple.Direction OUTTAKE_DIRECTION =
            DcMotorSimple.Direction.FORWARD;

    // Conversão dos dois motores paralelos
    public static final double SHOOTER_MOTOR_TICKS_PER_REV = 28.0;
    public static final double SHOOTER_MOTOR_NOMINAL_RPM = 3500.0;
    public static final double SHOOTER_MOTOR_REVS_PER_ROLLER_REV = 1.0;
    public static final double SHOOTER_TICKS_PER_ROLLER_REV =
            SHOOTER_MOTOR_TICKS_PER_REV * SHOOTER_MOTOR_REVS_PER_ROLLER_REV;

    // Conversão independente do motor de outtake.
    // Ajuste estes valores se a relação mecânica do outtake for diferente.
    public static final double OUTTAKE_MOTOR_TICKS_PER_REV = 28.0;
    public static final double OUTTAKE_MOTOR_NOMINAL_RPM = 3500.0;
    public static final double OUTTAKE_MOTOR_REVS_PER_ROLLER_REV = 1.0;
    public static final double OUTTAKE_TICKS_PER_ROLLER_REV =
            OUTTAKE_MOTOR_TICKS_PER_REV * OUTTAKE_MOTOR_REVS_PER_ROLLER_REV;

    // Shooter paralelo: limites e tolerância
    public static final double MIN_SHOOTER_RPM = 1000.0;
    public static final double MAX_SHOOTER_RPM = 6000.0;
    public static final double SHOOTER_READY_TOLERANCE_RPM = 100.0;

    // Outtake: limites e tolerância independentes
    public static final double MIN_OUTTAKE_RPM = 0.0;
    public static final double MAX_OUTTAKE_RPM = 6000.0;
    public static final double OUTTAKE_READY_TOLERANCE_RPM = 150.0;

    // RPM do par paralelo no modo manual (gamepad2 LB).
    public static final double MANUAL_SHOOTER_RPM = 6000.0;

    // RPM inicial sugerido para o outtake.
    // VALOR PROVISÓRIO: calibrar fisicamente com a bolinha e ajustar aqui.
    public static final double DEFAULT_OUTTAKE_RPM = 1500.0;

    // Fallback usado se a tabela de distância estiver inválida.
    public static final double DEFAULT_SHOOTER_RPM = MANUAL_SHOOTER_RPM;

    public static final long SHOOTER_SPINUP_DELAY_MS = 2000;

    public static final double SHOOTER_CALIBRATION_COARSE_STEP_RPM = 100.0;
    public static final double SHOOTER_CALIBRATION_FINE_STEP_RPM = 25.0;
}
