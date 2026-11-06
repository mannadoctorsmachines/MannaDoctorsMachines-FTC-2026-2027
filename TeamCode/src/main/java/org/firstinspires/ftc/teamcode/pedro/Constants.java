package org.firstinspires.ftc.teamcode.pedro;

import static java.lang.Double.NaN;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("rodaEsquerda");
        c.frontRightName.set("rodaDireita");
        c.backLeftName.set("trasEsquerda");
        c.backRightName.set("trasDireita");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odmetry");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(8.969434250996808E-4);
        c.yPodOffset.set(-0.0011394576235549657);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(3360929.09755779);
//                Controller secondaryTranslationalForward = Controller.proportional(1241773.4958681418);
//                Controller primaryTranslationalLateral = Controller.proportional(-0.29333131350665803);
//                Controller secondaryTranslationalLateral = Controller.proportional(-0.10837808238365954);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(4461.6267763331725));
//                c.brake.set(Controller.proportionalFeedforward(3792.3827598831967));
//
//                c.headingFeedback.set(Controller.proportional(3.6939213997471314));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.07134586617308084, 0.006615742884860267));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(-0.03154071992489577, 0.008161638772137447));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.004257756165481247, 0.002383178954957543));
//
//                c.maxAchievableForwardVelocity.set(53.348324658597164);
//                c.maxAchievableStrafeVelocity.set(15.964614501706928);
//                c.naturalForwardDeceleration.set(198.7467496879632);
//                c.naturalStrafeDeceleration.set(167.29323454290954);
//            }
//    );

//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(6.615732478807525);
//                Controller secondaryTranslationalForward = Controller.proportional(2.444336375291823);
//                Controller primaryTranslationalLateral = Controller.proportional(-4150424.1816401007);
//                Controller secondaryTranslationalLateral = Controller.proportional(-1533470.8337393838);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.030203575847191803));
//                c.brake.set(Controller.proportionalFeedforward(0.02567303947011303));
//
//                c.headingFeedback.set(Controller.proportional(4.391123061896489));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.061218064278943836, 0.007876624588414228));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.0837934552056662, 0.02129735842701238));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012413270981055818, 0.0016638658819482313));
//
//                c.maxAchievableForwardVelocity.set(52.984109197497204);
//                c.maxAchievableStrafeVelocity.set(51.085725399760385);
//                c.naturalForwardDeceleration.set(65.61277408701513);
//                c.naturalStrafeDeceleration.set(213.20810939224836);
//            }
//    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(NaN);
                Controller secondaryTranslationalForward = Controller.proportional(NaN);
                Controller primaryTranslationalLateral = Controller.proportional(611814.9829226164);
                Controller secondaryTranslationalLateral = Controller.proportional(226049.2881924777);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.8195536972618226));
                c.brake.set(Controller.proportionalFeedforward(0.6966206426725492));

                c.headingFeedback.set(Controller.proportional(4.145694449839484));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06606902841072393, 0.007843142733668915));

                c.linearBrakeCoefficients.set(Matrix.diag(0.03363383781047747, 0.014974471459325777));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0017048612420689938, 0.001986826090763435));

                c.maxAchievableForwardVelocity.set(58.64209827331572);
                c.maxAchievableStrafeVelocity.set(56.126041063673654);
                c.naturalForwardDeceleration.set(209.09386497723187);
                c.naturalStrafeDeceleration.set(114.271850996447);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}