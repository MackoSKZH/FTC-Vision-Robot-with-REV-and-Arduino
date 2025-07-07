package src;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.EnumSet;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Program")
public class alphaBot_OpMode_v02 extends LinearOpMode {

    private DcMotor LiftL;
    private DcMotor R;
    private DcMotor L;
    private Servo CatchL;
    private Servo CatchR;

    private DigitalChannel led;

    private static final double CATCH_OPEN_L = 0.3;
    private static final double CATCH_CLOSED_L = 0.7;

    private static final double CATCH_OPEN_R = 0.7;
    private static final double CATCH_CLOSED_R = 0.3;

    private PredominantColorProcessor.Builder myPredominantColorProcessorBuilder;
    private VisionPortal.Builder myVisionPortalBuilder;
    private PredominantColorProcessor myPredominantColorProcessor;
    private VisionPortal myVisionPortal;
    private PredominantColorProcessor.Result myResult;

    private EnumSet<PredominantColorProcessor.Swatch> container1;
    private EnumSet<PredominantColorProcessor.Swatch> container2;
    private EnumSet<PredominantColorProcessor.Swatch> container3;

    private AprilTagProcessor myAprilTagProcessor;
    private List<AprilTagDetection> currentDetections;

    private boolean closeStatus = false;

    private AprilTagDetection lastDetection;
    private PredominantColorProcessor.Swatch colorHold;

    private double lastKnownDistanceCm = -1;

    private int score;
    private long lastToggleTime;
    private boolean ledState = false;
    private long ledTimer = 0;

    private static final int COLOR_HISTORY_SIZE = 5;
    private PredominantColorProcessor.Swatch[] colorHistory = new PredominantColorProcessor.Swatch[COLOR_HISTORY_SIZE];
    private int colorHistoryIndex = 0;

    @Override
    public void runOpMode() {
        LiftL = hardwareMap.get(DcMotor.class, "Lift L");

        R = hardwareMap.get(DcMotor.class, "R");
        L = hardwareMap.get(DcMotor.class, "L");

        CatchL = hardwareMap.get(Servo.class, "CatchL");
        CatchR = hardwareMap.get(Servo.class, "CatchR");

        led = hardwareMap.get(DigitalChannel.class, "controlLed");
        led.setMode(DigitalChannel.Mode.OUTPUT);
        turnOffLed();

        LiftL.setDirection(DcMotor.Direction.REVERSE);
        R.setDirection(DcMotorSimple.Direction.REVERSE);

        myPredominantColorProcessorBuilder = new PredominantColorProcessor.Builder();
        myPredominantColorProcessorBuilder.setRoi(ImageRegion.asUnityCenterCoordinates(-0.2, -0.8, 0.2, -1.0));
        myPredominantColorProcessorBuilder.setSwatches(
                PredominantColorProcessor.Swatch.RED,
                PredominantColorProcessor.Swatch.ORANGE,
                PredominantColorProcessor.Swatch.YELLOW,
                PredominantColorProcessor.Swatch.GREEN,
                PredominantColorProcessor.Swatch.CYAN,
                PredominantColorProcessor.Swatch.BLUE,
                PredominantColorProcessor.Swatch.PURPLE,
                PredominantColorProcessor.Swatch.MAGENTA,
                PredominantColorProcessor.Swatch.WHITE,
                PredominantColorProcessor.Swatch.BLACK
        );

        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        myPredominantColorProcessor = myPredominantColorProcessorBuilder.build();
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.addProcessor(myPredominantColorProcessor);
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        myVisionPortalBuilder.setCameraResolution(new Size(320, 240));
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        myVisionPortal = myVisionPortalBuilder.build();

        telemetry.setMsTransmissionInterval(50);

        colorHold = null;
        score = 0;

        container1 = EnumSet.of(
                PredominantColorProcessor.Swatch.ORANGE,
                PredominantColorProcessor.Swatch.YELLOW
        );

        container2 = EnumSet.of(
                PredominantColorProcessor.Swatch.GREEN,
                PredominantColorProcessor.Swatch.CYAN,
                PredominantColorProcessor.Swatch.BLUE
        );

        container3 = EnumSet.of(
                PredominantColorProcessor.Swatch.PURPLE,
                PredominantColorProcessor.Swatch.RED
        );

        lastToggleTime = System.currentTimeMillis();
        ledState = false;
        led.setState(false);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                myResult = myPredominantColorProcessor.getAnalysis();
                currentDetections = myAprilTagProcessor.getDetections();

                if (!currentDetections.isEmpty()) {
                    lastDetection = currentDetections.get(0);

                    AprilTagPoseFtc pose = lastDetection.ftcPose;
                    pose = getPose(lastDetection, pose);

                    if (pose != null) {
                        lastKnownDistanceCm = pose.z;
                    }
                }

                controlRobot();
                displayTelemetry();
                telemetry.update();

                if (ledState && (System.currentTimeMillis() - ledTimer >= 2000)) {
                    led.setState(false);  // OFF
                    ledState = false;
                }

                if (myResult != null && myResult.closestSwatch != null) {
                    colorHistory[colorHistoryIndex] = myResult.closestSwatch;
                    colorHistoryIndex = (colorHistoryIndex + 1) % COLOR_HISTORY_SIZE;
                }

                sleep(20);
            }
        }
    }

    private void controlRobot() {
        double speedKoeficient = 0.3;

        L.setPower(gamepad1.left_stick_y * speedKoeficient);
        R.setPower(gamepad1.right_stick_y * speedKoeficient);

        if (gamepad1.dpad_down) {
            LiftL.setPower(0.3);
        } else if (gamepad1.dpad_up) {
            LiftL.setPower(-0.5);
        } else {
            LiftL.setPower(0);
        }

        if (gamepad1.right_trigger == 1) {
            CatchL.setPosition(CATCH_CLOSED_L);
            CatchR.setPosition(CATCH_CLOSED_R);
            if (colorHold == null) {
                colorHold = getMostFrequentColor();
            }
            closeStatus = true;
        } else {
            CatchL.setPosition(CATCH_OPEN_L);
            CatchR.setPosition(CATCH_OPEN_R);
            gameLogic();
            colorHold = null;
            closeStatus = false;
            lastDetection = null;
        }

        if (gamepad1.a) {
            displayTelemetry();
        } else {
            telemetry.clear();
        }
    }

    private void turnOnLed() {
        led.setState(false);
    }

    private void turnOffLed() {
        led.setState(true);
    }

    private void gameLogic() {
        if (colorHold != null && lastDetection != null) {
            int id = lastDetection.id;
            int lastScore = score;

            switch (id) {
                case 1:
                    if (container1.contains(colorHold)) score++;
                    break;
                case 2:
                    if (container2.contains(colorHold)) score++;
                    break;
                case 3:
                    if (container3.contains(colorHold)) score++;
                    break;
            }

            if (lastScore == score) {
                telemetry.addLine("Zly kontajner – bod nepridany");
                led.setState(false);
                ledState = false;
                ledTimer = System.currentTimeMillis();

            } else {
                telemetry.addLine("Spravny kontajner – bod pridany!");

                led.setState(true);
                ledState = true;
                ledTimer = System.currentTimeMillis();
            }

        }
    }

    private void displayTelemetry() {
        telemetry.addData("Vidim: ", myResult.closestSwatch);
        telemetry.addData("Drzana farba", colorHold != null ? colorHold.name() : "Žiadna");

        if (!currentDetections.isEmpty()) {
            AprilTagDetection tag = currentDetections.get(0);
            AprilTagPoseFtc pose = tag.ftcPose;

            pose = getPose(tag, pose);

            if (pose != null) {
                telemetry.addData("AprilTag ID", tag.id);
                telemetry.addData("Vzdialenost (Z cm)", JavaUtil.formatNumber(pose.z, 2, 1));
                if (pose.z < 15) {
                    telemetry.addLine("Sme pri kontajneri");
                } else {
                    telemetry.addLine("Kontajner daleko");
                }
            }

            String meaning = "Container: ";
            if (tag.id == 1) meaning += "RED";
            else if (tag.id == 2) meaning += "GREEN";
            else if (tag.id == 3) meaning += "DARK";
            else meaning = "Unknown container";
            telemetry.addLine(meaning);

        } else {
            telemetry.addLine("Ziadny AprilTag");
        }

        telemetry.addData("Body", score);
    }

    private PredominantColorProcessor.Swatch getMostFrequentColor() {
        java.util.Map<PredominantColorProcessor.Swatch, Integer> countMap = new java.util.HashMap<>();
        for (PredominantColorProcessor.Swatch swatch : colorHistory) {
            if (swatch != null && swatch != PredominantColorProcessor.Swatch.BLACK && swatch != PredominantColorProcessor.Swatch.WHITE) {
                countMap.put(swatch, countMap.getOrDefault(swatch, 0) + 1);
            }
        }

        PredominantColorProcessor.Swatch mostFrequent = null;
        int maxCount = 0;
        for (java.util.Map.Entry<PredominantColorProcessor.Swatch, Integer> entry : countMap.entrySet()) {
            if (entry.getValue() > maxCount) {
                mostFrequent = entry.getKey();
                maxCount = entry.getValue();
            }
        }

        return mostFrequent;
    }

    private AprilTagPoseFtc getPose(AprilTagDetection tag, AprilTagPoseFtc pose) {
        if (pose == null && tag.rawPose != null) {
            Orientation rot = Orientation.getOrientation(
                    tag.rawPose.R,
                    AxesReference.INTRINSIC,
                    AxesOrder.YXZ,
                    AngleUnit.DEGREES
            );

            double x = tag.rawPose.x;
            double y = tag.rawPose.z;
            double z = -tag.rawPose.y;

            double yaw = -rot.firstAngle;
            double roll = rot.thirdAngle;
            double pitch = rot.secondAngle;

            double range = Math.hypot(x, y);
            double bearing = Math.toDegrees(Math.atan2(-x, y));
            double elevation = Math.toDegrees(Math.atan2(z, y));

            return new AprilTagPoseFtc(x, y, z, yaw, pitch, roll, range, bearing, elevation);
        }
        return null;
    }
}