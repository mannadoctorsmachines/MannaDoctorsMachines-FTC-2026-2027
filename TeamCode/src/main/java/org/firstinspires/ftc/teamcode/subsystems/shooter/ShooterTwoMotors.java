package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.RConstants;
import org.firstinspires.ftc.teamcode.util.MathU;

/**
 * Shooter com três motores:
 * - esquerdo e direito: trabalham em paralelo, sempre com o mesmo RPM;
 * - outtake: possui RPM independente e normalmente menor.
 */
public class ShooterTwoMotors {

    private DcMotorEx shooterMotorLeft;
    private DcMotorEx shooterMotorRight;
    private DcMotorEx shooterOuttakeMotor;

    private double targetRPM = 0.0;
    private double targetOuttakeRPM = 0.0;

    public void init(HardwareMap hardwareMap) {
        // Os três motores são obrigatórios na configuração de competição.
        shooterMotorLeft = hardwareMap.get(
                DcMotorEx.class,
                ShooterConfig.SHOOTER_LEFT
        );

        shooterMotorRight = hardwareMap.get(
                DcMotorEx.class,
                ShooterConfig.SHOOTER_RIGHT
        );

        shooterOuttakeMotor = hardwareMap.get(
                DcMotorEx.class,
                ShooterConfig.SHOOTER_OUTTAKE
        );

        /*
         * Preserve as direções já utilizadas nos dois motores paralelos.
         * A direção do outtake deve ser confirmada no teste de bancada:
         * ela depende da montagem física e do sentido necessário para puxar
         * a bolinha.
         */
        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterOuttakeMotor.setDirection(ShooterConfig.OUTTAKE_DIRECTION);

        configureMotor(shooterMotorLeft);
        configureMotor(shooterMotorRight);
        configureMotor(shooterOuttakeMotor);
    }

    private void configureMotor(DcMotorEx motor) {
        if (motor == null) {
            return;
        }

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Zera os encoders dos três motores com o sistema parado. */
    public void resetEncoders() {
        stop();
        configureMotor(shooterMotorLeft);
        configureMotor(shooterMotorRight);
        configureMotor(shooterOuttakeMotor);
    }

    /**
     * Mantém compatibilidade com os OpModes atuais.
     * Aplica o RPM solicitado aos dois motores paralelos e o RPM padrão
     * configurado para o outtake.
     */
    public void setRPM(double parallelRPM) {
        setRPM(parallelRPM, ShooterConfig.DEFAULT_OUTTAKE_RPM);
    }

    /**
     * Define os RPMs separadamente.
     * Os motores esquerdo e direito sempre recebem o mesmo alvo.
     */
    public void setRPM(double parallelRPM, double outtakeRPM) {
        if (!hasValidEncoderConversion()
                || !hasNominalMotorRPMConfigured()
                || !hasValidOuttakeEncoderConversion()
                || !hasOuttakeNominalMotorRPMConfigured()) {
            stop();
            return;
        }

        double maximumParallelRPM = getMaximumAllowedRollerRPM();
        double minimumParallelRPM = Math.min(
                ShooterConfig.MIN_SHOOTER_RPM,
                maximumParallelRPM
        );

        targetRPM = MathU.clamp(
                parallelRPM,
                minimumParallelRPM,
                maximumParallelRPM
        );

        double maximumOuttakeRPM = getMaximumAllowedOuttakeRPM();
        double minimumOuttakeRPM = Math.min(
                ShooterConfig.MIN_OUTTAKE_RPM,
                maximumOuttakeRPM
        );

        targetOuttakeRPM = MathU.clamp(
                outtakeRPM,
                minimumOuttakeRPM,
                maximumOuttakeRPM
        );

        double parallelTicksPerSecond = parallelRPMToTicksPerSecond(targetRPM);
        double outtakeTicksPerSecond = outtakeRPMToTicksPerSecond(targetOuttakeRPM);

        // Os dois motores paralelos recebem exatamente o mesmo comando.
        shooterMotorLeft.setVelocity(parallelTicksPerSecond);
        shooterMotorRight.setVelocity(parallelTicksPerSecond);

        // O outtake recebe seu próprio alvo, independente do par.
        shooterOuttakeMotor.setVelocity(outtakeTicksPerSecond);
    }

    /** Aplica a mesma potência bruta aos três motores, para testes manuais. */
    public void setRawPower(double power) {
        double safePower = MathU.clamp(
                power,
                RConstants.MIN_MOTOR_POWER,
                RConstants.MAX_MOTOR_POWER
        );

        targetRPM = 0.0;
        targetOuttakeRPM = 0.0;

        shooterMotorLeft.setPower(safePower);
        shooterMotorRight.setPower(safePower);
        shooterOuttakeMotor.setPower(safePower);
    }

    public void stop() {
        targetRPM = 0.0;
        targetOuttakeRPM = 0.0;

        stopMotor(shooterMotorLeft);
        stopMotor(shooterMotorRight);
        stopMotor(shooterOuttakeMotor);
    }

    private void stopMotor(DcMotorEx motor) {
        if (motor != null) {
            motor.setVelocity(0.0);
            motor.setPower(0.0);
        }
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getTargetOuttakeRPM() {
        return targetOuttakeRPM;
    }

    public double getLeftRPM() {
        return getParallelRollerRPM(shooterMotorLeft);
    }

    public double getRightRPM() {
        return getParallelRollerRPM(shooterMotorRight);
    }

    public double getOuttakeRPM() {
        return getOuttakeRollerRPM(shooterOuttakeMotor);
    }

    /** Média dos dois rolos paralelos; não inclui o outtake. */
    public double getCurrentRPM() {
        return (getLeftRPM() + getRightRPM()) / 2.0;
    }

    public double getLeftMotorRPM() {
        return getParallelMotorRPM(shooterMotorLeft);
    }

    public double getRightMotorRPM() {
        return getParallelMotorRPM(shooterMotorRight);
    }

    public double getOuttakeMotorRPM() {
        return getOuttakeMotorRPMValue(shooterOuttakeMotor);
    }

    public double getLeftTicksPerSecond() {
        return getTicksPerSecond(shooterMotorLeft);
    }

    public double getRightTicksPerSecond() {
        return getTicksPerSecond(shooterMotorRight);
    }

    public double getOuttakeTicksPerSecond() {
        return getTicksPerSecond(shooterOuttakeMotor);
    }

    public int getLeftEncoderPosition() {
        return getEncoderPosition(shooterMotorLeft);
    }

    public int getRightEncoderPosition() {
        return getEncoderPosition(shooterMotorRight);
    }

    public int getOuttakeEncoderPosition() {
        return getEncoderPosition(shooterOuttakeMotor);
    }

    public boolean hasRightMotor() {
        return shooterMotorRight != null;
    }

    public boolean hasOuttakeMotor() {
        return shooterOuttakeMotor != null;
    }

    /**
     * O shooter só está pronto quando os três motores atingem seus
     * respectivos alvos. Isso evita liberar a alimentação se o outtake
     * ainda estiver muito distante da velocidade configurada.
     */
    public boolean isAtTargetRPM() {
        if (targetRPM <= 0.0) {
            return false;
        }

        boolean leftReady = Math.abs(getLeftRPM() - targetRPM)
                <= ShooterConfig.SHOOTER_READY_TOLERANCE_RPM;

        boolean rightReady = Math.abs(getRightRPM() - targetRPM)
                <= ShooterConfig.SHOOTER_READY_TOLERANCE_RPM;

        boolean outtakeReady = targetOuttakeRPM <= 0.0
                || Math.abs(getOuttakeRPM() - targetOuttakeRPM)
                <= ShooterConfig.OUTTAKE_READY_TOLERANCE_RPM;

        return leftReady && rightReady && outtakeReady;
    }

    private double parallelRPMToTicksPerSecond(double rpm) {
        return rpm * ShooterConfig.SHOOTER_TICKS_PER_ROLLER_REV / 60.0;
    }

    private double outtakeRPMToTicksPerSecond(double rpm) {
        return rpm * ShooterConfig.OUTTAKE_TICKS_PER_ROLLER_REV / 60.0;
    }

    private double getTicksPerSecond(DcMotorEx motor) {
        return motor == null ? 0.0 : Math.abs(motor.getVelocity());
    }

    private int getEncoderPosition(DcMotorEx motor) {
        return motor == null ? 0 : motor.getCurrentPosition();
    }

    private double getParallelMotorRPM(DcMotorEx motor) {
        if (ShooterConfig.SHOOTER_MOTOR_TICKS_PER_REV <= 0.0) {
            return 0.0;
        }

        return getTicksPerSecond(motor) * 60.0
                / ShooterConfig.SHOOTER_MOTOR_TICKS_PER_REV;
    }

    private double getOuttakeMotorRPMValue(DcMotorEx motor) {
        if (ShooterConfig.OUTTAKE_MOTOR_TICKS_PER_REV <= 0.0) {
            return 0.0;
        }

        return getTicksPerSecond(motor) * 60.0
                / ShooterConfig.OUTTAKE_MOTOR_TICKS_PER_REV;
    }

    private double getParallelRollerRPM(DcMotorEx motor) {
        if (!hasValidEncoderConversion()) {
            return 0.0;
        }

        return getParallelMotorRPM(motor)
                / ShooterConfig.SHOOTER_MOTOR_REVS_PER_ROLLER_REV;
    }

    private double getOuttakeRollerRPM(DcMotorEx motor) {
        if (!hasValidOuttakeEncoderConversion()) {
            return 0.0;
        }

        return getOuttakeMotorRPMValue(motor)
                / ShooterConfig.OUTTAKE_MOTOR_REVS_PER_ROLLER_REV;
    }

    public double getMaximumAllowedRollerRPM() {
        double configuredMaximum = ShooterConfig.MAX_SHOOTER_RPM;

        if (ShooterConfig.SHOOTER_MOTOR_NOMINAL_RPM <= 0.0
                || ShooterConfig.SHOOTER_MOTOR_REVS_PER_ROLLER_REV <= 0.0) {
            return configuredMaximum;
        }

        double nominalRollerRPM = ShooterConfig.SHOOTER_MOTOR_NOMINAL_RPM
                / ShooterConfig.SHOOTER_MOTOR_REVS_PER_ROLLER_REV;

        return Math.min(configuredMaximum, nominalRollerRPM);
    }

    public double getMaximumAllowedOuttakeRPM() {
        double configuredMaximum = ShooterConfig.MAX_OUTTAKE_RPM;

        if (ShooterConfig.OUTTAKE_MOTOR_NOMINAL_RPM <= 0.0
                || ShooterConfig.OUTTAKE_MOTOR_REVS_PER_ROLLER_REV <= 0.0) {
            return configuredMaximum;
        }

        double nominalRollerRPM = ShooterConfig.OUTTAKE_MOTOR_NOMINAL_RPM
                / ShooterConfig.OUTTAKE_MOTOR_REVS_PER_ROLLER_REV;

        return Math.min(configuredMaximum, nominalRollerRPM);
    }

    public double getNominalRollerRPM() {
        if (ShooterConfig.SHOOTER_MOTOR_NOMINAL_RPM <= 0.0
                || ShooterConfig.SHOOTER_MOTOR_REVS_PER_ROLLER_REV <= 0.0) {
            return 0.0;
        }

        return ShooterConfig.SHOOTER_MOTOR_NOMINAL_RPM
                / ShooterConfig.SHOOTER_MOTOR_REVS_PER_ROLLER_REV;
    }

    public double getNominalOuttakeRPM() {
        if (ShooterConfig.OUTTAKE_MOTOR_NOMINAL_RPM <= 0.0
                || ShooterConfig.OUTTAKE_MOTOR_REVS_PER_ROLLER_REV <= 0.0) {
            return 0.0;
        }

        return ShooterConfig.OUTTAKE_MOTOR_NOMINAL_RPM
                / ShooterConfig.OUTTAKE_MOTOR_REVS_PER_ROLLER_REV;
    }

    public boolean hasNominalMotorRPMConfigured() {
        return ShooterConfig.SHOOTER_MOTOR_NOMINAL_RPM > 0.0;
    }

    public boolean hasOuttakeNominalMotorRPMConfigured() {
        return ShooterConfig.OUTTAKE_MOTOR_NOMINAL_RPM > 0.0;
    }

    public boolean hasValidEncoderConversion() {
        return ShooterConfig.SHOOTER_MOTOR_TICKS_PER_REV > 0.0
                && ShooterConfig.SHOOTER_MOTOR_REVS_PER_ROLLER_REV > 0.0
                && ShooterConfig.SHOOTER_TICKS_PER_ROLLER_REV > 0.0;
    }

    public boolean hasValidOuttakeEncoderConversion() {
        return ShooterConfig.OUTTAKE_MOTOR_TICKS_PER_REV > 0.0
                && ShooterConfig.OUTTAKE_MOTOR_REVS_PER_ROLLER_REV > 0.0
                && ShooterConfig.OUTTAKE_TICKS_PER_ROLLER_REV > 0.0;
    }
}
