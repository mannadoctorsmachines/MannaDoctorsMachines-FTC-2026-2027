package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.RConstants;
import org.firstinspires.ftc.teamcode.util.MathU;

public class ShooterTwoMotors {

    /*
     * O motor esquerdo é o motor principal.
     *
     * O motor direito é opcional:
     * - 2 motores disponíveis -> usa os dois;
     * - somente o esquerdo disponível -> usa somente o esquerdo.
     */
    private DcMotorEx shooterMotorLeft;
    private DcMotorEx shooterMotorRight;
    private boolean hasRightMotor = false;
    private double leftTargetRPM = 0.0;
    private double rightTargetRPM = 0.0;

    public void init(HardwareMap hardwareMap) {

        /*
         * O motor esquerdo continua sendo obrigatório.
         */
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, ShooterConfig.SHOOTER_LEFT);

        /*
         * O motor direito é opcional.
         *
         * Se ele não estiver configurado no Robot Configuration,
         * o Shooter continua funcionando com apenas um motor.
         */
        shooterMotorRight = null;
        hasRightMotor = false;

        if (hardwareMap.tryGet(DcMotorEx.class, ShooterConfig.SHOOTER_RIGHT) != null) {

            shooterMotorRight = hardwareMap.tryGet(DcMotorEx.class, ShooterConfig.SHOOTER_RIGHT);
            hasRightMotor = true;
        }

        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        configureMotor(shooterMotorLeft);

        if (shooterMotorRight != null) {

            shooterMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
            configureMotor(shooterMotorRight);
        }
    }

    private void configureMotor(DcMotorEx motor) {

        if (motor == null) {
            return;
        }

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Zera somente os encoders disponíveis do shooter.
     */
    public void resetEncoders() {

        stop();

        configureMotor(shooterMotorLeft);
        configureMotor(shooterMotorRight);
    }

    /**
     * Define um RPM-base mantendo a proporção de teste entre os motores:
     * esquerdo = 100%; direito = 90%.
     * Usado pelos modos manual e pelos cálculos de distância do autónomo.
     */
    public void setRPM(double rpm) {
        setRPM(rpm, rpm * ShooterConfig.RIGHT_SHOOTER_RPM_RATIO);
    }

    /** Define alvos independentes para cada motor (RPM do rolo). */
    public void setRPM(double leftRPM, double rightRPM) {

        if (!hasValidEncoderConversion() || !hasNominalMotorRPMConfigured()) {
            stop();
            return;
        }

        double maximumAllowedRPM = getMaximumAllowedRollerRPM();
        double minimumAllowedRPM = Math.min(ShooterConfig.MIN_SHOOTER_RPM, maximumAllowedRPM);

        leftTargetRPM = MathU.clamp(leftRPM, minimumAllowedRPM, maximumAllowedRPM);
        rightTargetRPM = shooterMotorRight == null
                ? 0.0
                : MathU.clamp(rightRPM, minimumAllowedRPM, maximumAllowedRPM);

        shooterMotorLeft.setVelocity(rpmToTicksPerSecond(leftTargetRPM));

        if (shooterMotorRight != null) {
            shooterMotorRight.setVelocity(rpmToTicksPerSecond(rightTargetRPM));
        }
    }

    public void setReverseRPM(double rpm) {

        if (!hasValidEncoderConversion() || !hasNominalMotorRPMConfigured()) {
            stop();
            return;
        }

        double maximumAllowedRPM = getMaximumAllowedRollerRPM();
        double minimumAllowedRPM =
                Math.min(ShooterConfig.MIN_SHOOTER_RPM, maximumAllowedRPM);

        double reverseRPM = MathU.clamp(
                Math.abs(rpm),
                minimumAllowedRPM,
                maximumAllowedRPM
        );

        leftTargetRPM = reverseRPM;

        rightTargetRPM = shooterMotorRight == null
                ? 0.0
                : reverseRPM;

        shooterMotorLeft.setVelocity(
                -rpmToTicksPerSecond(reverseRPM)
        );

        if (shooterMotorRight != null) {
            shooterMotorRight.setVelocity(
                    -rpmToTicksPerSecond(reverseRPM)
            );
        }
    }

    public void setRawPower(double power) {

        double safePower = MathU.clamp(power, RConstants.MIN_MOTOR_POWER, RConstants.MAX_MOTOR_POWER);

        leftTargetRPM = 0.0;
        rightTargetRPM = 0.0;

        shooterMotorLeft.setPower(safePower);

        if (shooterMotorRight != null) {
            shooterMotorRight.setPower(safePower);
        }
    }

    public void stop() {

        leftTargetRPM = 0.0;
        rightTargetRPM = 0.0;

        stopMotor(shooterMotorLeft);
        stopMotor(shooterMotorRight);
    }

    private void stopMotor(DcMotorEx motor) {
        if (motor != null) {
            motor.setVelocity(0.0);
            motor.setPower(0.0);
        }
    }

    public double getTargetRPM() {
        return leftTargetRPM;
    }

    public double getLeftRPM() {
        return getRollerRPM(shooterMotorLeft);
    }

    public double getRightRPM() {
        if (shooterMotorRight == null) {
            return 0.0;
        }

        return getRollerRPM(shooterMotorRight);
    }

    /**
     * Retorna o RPM real do shooter.
     *
     * Com dois motores:
     * média dos dois.
     *
     * Com um motor:
     * RPM do motor esquerdo.
     */
    public double getCurrentRPM() {
        double leftRPM = getLeftRPM();

        if (shooterMotorRight == null) {
            return leftRPM;
        }

        double rightRPM = getRightRPM();

        return (leftRPM + rightRPM) / 2.0;
    }

    /**
     * RPM real do eixo de saída
     * do motor esquerdo.
     */
    public double getLeftMotorRPM() {

        return getMotorRPM(
                shooterMotorLeft
        );
    }

    /**
     * RPM real do eixo de saída
     * do motor direito.
     *
     * Retorna 0 se o segundo motor
     * não estiver presente.
     */
    public double getRightMotorRPM() {
        if (shooterMotorRight == null) {
            return 0.0;
        }

        return getMotorRPM(shooterMotorRight);
    }

    /**
     * Ticks por segundo crus
     * do encoder esquerdo.
     */
    public double getLeftTicksPerSecond() {
        return getTicksPerSecond(shooterMotorLeft);
    }

    /**
     * Ticks por segundo crus
     * do encoder direito.
     */
    public double getRightTicksPerSecond() {
        if (shooterMotorRight == null) {
            return 0.0;
        }

        return getTicksPerSecond(shooterMotorRight);
    }

    public int getLeftEncoderPosition() {
        return shooterMotorLeft == null ? 0 : shooterMotorLeft.getCurrentPosition();
    }

    public int getRightEncoderPosition() {
        return shooterMotorRight == null ? 0 : shooterMotorRight.getCurrentPosition();
    }

    /**
     * Indica se o segundo motor está disponível.
     */
    public boolean hasRightMotor() {
        return hasRightMotor && shooterMotorRight != null;
    }

    private double getTicksPerSecond(DcMotorEx motor) {
        if (motor == null) {
            return 0.0;
        }

        /*
         * abs evita valor negativo no motor
         * cuja direção foi invertida.
         */
        return Math.abs(motor.getVelocity());
    }

    private double getMotorRPM(DcMotorEx motor) {

        if (ShooterConfig.SHOOTER_MOTOR_TICKS_PER_REV <= 0.0) {
            return 0.0;
        }

        return (getTicksPerSecond(motor) * 60.0) / ShooterConfig.SHOOTER_MOTOR_TICKS_PER_REV;
    }

    private double getRollerRPM(DcMotorEx motor) {
        if (!hasValidEncoderConversion()) {
            return 0.0;
        }

        return getMotorRPM(motor) / ShooterConfig.SHOOTER_MOTOR_REVS_PER_ROLLER_REV;
    }

    /**
     * Considera o shooter pronto somente
     * quando todos os motores disponíveis
     * estiverem dentro da tolerância.
     */
    public boolean isAtTargetRPM() {

        if (leftTargetRPM <= 0.0) {
            return false;
        }

        double leftError = Math.abs(getLeftRPM() - leftTargetRPM);

        boolean leftReady = leftError <= ShooterConfig.SHOOTER_READY_TOLERANCE_RPM;
        /*
         * Se houver apenas um motor,
         * somente o esquerdo é considerado.
         */
        if (shooterMotorRight == null) {
            return leftReady;
        }

        double rightError = Math.abs(getRightRPM() - rightTargetRPM);

        boolean rightReady = rightError <= ShooterConfig.SHOOTER_READY_TOLERANCE_RPM;

        return leftReady && rightReady;
    }

    private double rpmToTicksPerSecond(
            double rpm
    ) {

        return rpm * ShooterConfig.SHOOTER_TICKS_PER_ROLLER_REV / 60.0;
    }

    /**
     * Maior RPM do rolo que o código aceitará.
     *
     * Quando o RPM nominal do motor estiver
     * preenchido, o limite também respeita
     * a transmissão das polias.
     */
    public double getMaximumAllowedRollerRPM() {

        double configuredMaximum = ShooterConfig.MAX_SHOOTER_RPM;

        if (ShooterConfig.SHOOTER_MOTOR_NOMINAL_RPM <= 0.0 || ShooterConfig.SHOOTER_MOTOR_REVS_PER_ROLLER_REV <= 0.0) {
            return configuredMaximum;
        }

        double nominalRollerRPM = ShooterConfig.SHOOTER_MOTOR_NOMINAL_RPM / ShooterConfig.SHOOTER_MOTOR_REVS_PER_ROLLER_REV;

        return Math.min(
                configuredMaximum,
                nominalRollerRPM
        );
    }

    public double getNominalRollerRPM() {

        if (ShooterConfig.SHOOTER_MOTOR_NOMINAL_RPM
                <= 0.0
                || ShooterConfig.SHOOTER_MOTOR_REVS_PER_ROLLER_REV
                <= 0.0) {
            return 0.0;
        }

        return ShooterConfig.SHOOTER_MOTOR_NOMINAL_RPM
                / ShooterConfig.SHOOTER_MOTOR_REVS_PER_ROLLER_REV;
    }

    public boolean hasNominalMotorRPMConfigured() {

        return ShooterConfig.SHOOTER_MOTOR_NOMINAL_RPM > 0.0;
    }

    public boolean hasValidEncoderConversion() {

        return ShooterConfig.SHOOTER_MOTOR_TICKS_PER_REV > 0.0
                && ShooterConfig.SHOOTER_MOTOR_REVS_PER_ROLLER_REV
                > 0.0
                && ShooterConfig.SHOOTER_TICKS_PER_ROLLER_REV
                > 0.0;
    }

    /**
     * RPM-alvo do motor esquerdo.
     */
    public double getLeftTargetRPM() {
        return leftTargetRPM;
    }

    /**
     * RPM-alvo do motor direito.
     */
    public double getRightTargetRPM() {
        return rightTargetRPM;
    }

}