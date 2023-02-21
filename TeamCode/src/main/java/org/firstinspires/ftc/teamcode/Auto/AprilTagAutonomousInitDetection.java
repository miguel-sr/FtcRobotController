/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AprilTagAutonomousInitDetection extends LinearOpMode {

    DcMotor leftFront, leftRear, rightFront, rightRear;
    DcMotor slide;

    Servo dropper;

    double COUNTS_PER_MOTOR_REV = 21;
    double DRIVE_GEAR_REDUCTION = 20.8;
    double DIAMETRO_RODA = 9.6;
    double circunferencia = DIAMETRO_RODA * Math.PI;

    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;

    Orientation angles;

    OpenCvCamera camera;
    Pipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dropper = hardwareMap.get(Servo.class, "servo");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); // Configuração do sensor de giroscópio
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new Pipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline); //setup da câmera para identificar somente o pipeline
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT); //rastreamento do objeto
            }

            @Override
            public void onError(int errorCode) {}
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if (tagOfInterest == null || tagOfInterest.id == LEFT){

        } else if (tagOfInterest.id == MIDDLE){

        } else {
            MOVE_WITH_ENCODERS(leftFront, leftRear, rightFront, rightRear, imu, 200, 1, 0, 1.5 ,"F");
            MOVE_WITH_ENCODERS(leftFront, leftRear, rightFront, rightRear, imu, 150, 0.7, 45, 3 ,"F");
            SET_SLIDE_POSITION(slide, 0.8, "7");
            MOVE_WITH_ENCODERS(leftFront, leftRear, rightFront, rightRear, imu, 50, 0.5, 0, 1.5 ,"F");
            dropper.setPosition(0.6);
            sleep(500);
            dropper.setPosition(0);
            MOVE_WITH_ENCODERS(leftFront, leftRear, rightFront, rightRear, imu, 50, 0.5, 0, 1.5 ,"T");
            SET_SLIDE_POSITION(slide, 0.8, "1");
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("angle", angles);
    }


    public void MOVE_WITH_ENCODERS(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear, BNO055IMU imu,
                                   double target, double power, double angle, double timeout, String direction) {

        int targetPosition = (int)Math.round(target * (DRIVE_GEAR_REDUCTION * COUNTS_PER_MOTOR_REV) / circunferencia);

        SET_MODE_STOP_AND_RESET_ENCODER(leftFront, leftRear, rightFront, rightRear);
        SET_MODE_RUN_USING_ENCODER(leftFront, leftRear, rightFront, rightRear);
        SET_MODE_STOP_AND_RESET_ENCODER(leftFront, leftRear, rightFront, rightRear);

        if (opModeIsActive()) {
            switch (direction) {
                case "F":
                    leftFront.setTargetPosition(targetPosition);
                    leftRear.setTargetPosition(targetPosition);
                    rightFront.setTargetPosition(targetPosition);
                    rightRear.setTargetPosition(targetPosition);

                    SET_MODE_RUN_TO_POSITION(leftFront, leftRear, rightFront, rightRear);

                    RUN_MOTORS_WITH_IMU_PID_CONTROLLER(leftFront, leftRear, rightFront, rightRear, imu, power, angle, timeout);

                    SET_MODE_STOP_AND_RESET_ENCODER(leftFront, leftRear, rightFront, rightRear);
                    break;

                case "T":
                    leftFront.setTargetPosition(-targetPosition);
                    leftRear.setTargetPosition(-targetPosition);
                    rightFront.setTargetPosition(-targetPosition);
                    rightRear.setTargetPosition(-targetPosition);

                    SET_MODE_RUN_TO_POSITION(leftFront, leftRear, rightFront, rightRear);

                    RUN_MOTORS_WITH_IMU_PID_CONTROLLER(leftFront, leftRear, rightFront, rightRear, imu, power, angle, timeout);

                    SET_MODE_STOP_AND_RESET_ENCODER(leftFront, leftRear, rightFront, rightRear);
                    break;

                case "E":
                    leftFront.setTargetPosition(-targetPosition);
                    leftRear.setTargetPosition(targetPosition);
                    rightFront.setTargetPosition(targetPosition);
                    rightRear.setTargetPosition(-targetPosition);

                    SET_MODE_RUN_TO_POSITION(leftFront, leftRear, rightFront, rightRear);

                    runtime.reset();

                    while (opModeIsActive() && (runtime.seconds() < timeout) &&
                            (leftFront.isBusy() && leftRear.isBusy() && rightFront.isBusy() && rightRear.isBusy())) {

                        leftFront.setPower(Math.abs(power));
                        leftRear.setPower(Math.abs(power));
                        rightFront.setPower(Math.abs(power));
                        rightRear.setPower(Math.abs(power));

                    }

                    SET_MODE_STOP_AND_RESET_ENCODER(leftFront, leftRear, rightFront, rightRear);
                    break;

                case "D":
                    leftFront.setTargetPosition(targetPosition);
                    leftRear.setTargetPosition(-targetPosition);
                    rightFront.setTargetPosition(-targetPosition);
                    rightRear.setTargetPosition(targetPosition);

                    SET_MODE_RUN_TO_POSITION(leftFront, leftRear, rightFront, rightRear);

                    runtime.reset();

                    while (opModeIsActive() && (runtime.seconds() < timeout) &&
                            (leftFront.isBusy() && leftRear.isBusy() && rightFront.isBusy() && rightRear.isBusy())) {
                        leftFront.setPower(Math.abs(power));
                        leftRear.setPower(Math.abs(power));
                        rightFront.setPower(Math.abs(power));
                        rightRear.setPower(Math.abs(power));
                    }

                    SET_MODE_STOP_AND_RESET_ENCODER(leftFront, leftRear, rightFront, rightRear);
                    break;
            }
        }
    }

    public void TURN_WITH_IMU_PID_CONTROLLER(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear, BNO055IMU imu,
                                             double angle, double power, double timeout) {
        double Kp = 0.02;
        double Ki = 0.001;
        double Kd = 0;

        double integralSum = 0;
        double integralSumLimit = 0.25;

        double lastError = 0;

        double lastReference = angle;

        double a = 0.8; // 0 < a < 1
        double currentFilterEstimate;
        double previousFilterEstimate = 0;

        ElapsedTime timer = new ElapsedTime();

        runtime.reset();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double z = Math.round(angles.firstAngle);

        while (opModeIsActive() && (runtime.seconds() < timeout) && (z != angle)) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            z = Math.round(angles.firstAngle);

            double error = angle - z;
            double errorChange = (error - lastError);

            currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
            previousFilterEstimate = currentFilterEstimate;

            double derivative = currentFilterEstimate  / timer.seconds();

            integralSum = integralSum + (error * timer.seconds());

            if (integralSum > integralSumLimit) {
                integralSum = integralSumLimit;
            }
            if (integralSum < -integralSumLimit) {
                integralSum = -integralSumLimit;
            }

            if (angle != lastReference) {
                integralSum = 0;
            }

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            if (out > power) {
                out = power;
            }
            if (out < -power) {
                out = -power;
            }

            leftFront.setPower(out);
            leftRear.setPower(out);
            rightFront.setPower(-out);
            rightRear.setPower(-out);

            lastError = error;
            lastReference = angle;

            timer.reset();
        }

        SET_MODE_STOP_AND_RESET_ENCODER(leftFront, leftRear, rightFront, rightRear);
    }

    public void RUN_MOTORS_WITH_IMU_PID_CONTROLLER(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear, BNO055IMU imu, double power, double angle, double timeout) {

        double Kp = 0.02;
        double Ki = 0;
        double Kd = 0;

        double integralSum = 0;
        double integralSumLimit = 0.25;

        double outLimit = 0.3;

        double lastError = 0;

        double lastReference = angle;

        double a = 0.8; // 0 < a < 1
        double currentFilterEstimate;
        double previousFilterEstimate = 0;

        ElapsedTime timer = new ElapsedTime();

        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < timeout) &&
                (leftFront.isBusy() && leftRear.isBusy() && rightFront.isBusy() && rightRear.isBusy())) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double error = angle - Math.round(angles.firstAngle);
            double errorChange = (error - lastError);

            currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
            previousFilterEstimate = currentFilterEstimate;

            double derivative = currentFilterEstimate  / timer.seconds();

            integralSum = integralSum + (error * timer.seconds());

            if (integralSum > integralSumLimit) {
                integralSum = integralSumLimit;
            }
            if (integralSum < -integralSumLimit) {
                integralSum = -integralSumLimit;
            }

            if (angle != lastReference) {
                integralSum = 0;
            }

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            if (out > outLimit) {
                out = outLimit;
            }
            if (out < -outLimit) {
                out = -outLimit;
            }

            leftFront.setPower(power + out);
            leftRear.setPower(power + out);
            rightFront.setPower(power - out);
            rightRear.setPower(power - out);

            lastError = error;
            lastReference = angle;

            timer.reset();
        }
    }

    public void SET_MODE_STOP_AND_RESET_ENCODER(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void SET_MODE_RUN_USING_ENCODER(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear) {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void SET_MODE_RUN_TO_POSITION(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear) {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void SET_SLIDE_POSITION(DcMotor motor, double power, String level) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        switch (level) {
            case "1":
                SLIDE(motor, 0, power);
                break;
            case "2":
                SLIDE(motor, 200, power);
                break;
            case "3":
                SLIDE(motor, 300, power);
                break;
            case "4":
                SLIDE(motor, 400, power);
                break;
            case "5":
                SLIDE(motor, 500, power);
                break;
            case "6":
                SLIDE(motor, 600, power);
                break;
            case "7":
                SLIDE(motor, 2880, power);
                break;
        }
    }

    public void SLIDE(DcMotor motor, int position, double power) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (motor.isBusy()) {
            motor.setPower(power);
        } else {
            motor.setPower(0);
        }
    }
}