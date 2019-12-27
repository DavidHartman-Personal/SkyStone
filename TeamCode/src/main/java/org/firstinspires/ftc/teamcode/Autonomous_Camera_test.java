package org.firstinspires.ftc.teamcode;

import android.text.TextUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;


@Autonomous(name="Autonomous_Camera_test", group="Autonomous Opmode")
//@Disabled
public class Autonomous_Camera_test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private Servo servoTest = null;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    //Log file members
    private PrintStream visionLog = null;
    private String visionLogFileBaseName = "/sdcard/FIRST/autonomous_camera_test_vision_log";
    private SimpleDateFormat visionLogFileTimestamp = new SimpleDateFormat("yyyyMMddHHmmss", Locale.US);
    private static String milliSecondDateTimeFormatVisionLog = "MM/dd/YYYY HH:mm:ss.SSS";
    private static SimpleDateFormat visionLogDateTimeFormat = new SimpleDateFormat(milliSecondDateTimeFormatVisionLog, Locale.US);

    private static List<String> visionLogHeaderColumns = Arrays.asList(
            "time",
            "object_label",
            "image_height",
            "object_height",
            "image_width",
            "object_width",
            "object_angle",
            "left",
            "right",
            "top",
            "bottom",
            "confidence",
            "left_power",
            "right_power",
            "heading");
    private static String visionLogHeader = TextUtils.join("\t",visionLogHeaderColumns);

    private static final String VUFORIA_KEY =
            "ARnmHRP/////AAABmelMKT/JJEYBgJG2zP2N1mSKxQ08h1XHpnJGbaExGdkvf5np7T2lHruMd57XOaTTdLmerBqKU9uk+2Lgk4pOCLccpz5IVDIcnG33p4k0d+B1CFdOVP/4kVK/DZQUhN8Vl0hYFU8jeNfC47Hit3efPNoh/gn5mxDqSTuWPLDOHa+FpAjW5shVIqk84gM2NFrJO656DqwjTW1nWNRXxkUl4FeBubo6WDkgXsMBNEpbjd0MwsahDOQAKE1s6Bm1ALLYB4rDOfUdF10y3Ynw4ZgvfI8Q6oCuOL3pEHLyrwobaLz17qRfNsJOWB/tOqaIabXlTqj7lDF5DzWLVr+uM45p2iWWi/BT4DmrH72GsHs9PyEj";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (tfod != null) {
            tfod.activate();
        }
        leftMotor  = hardwareMap.get(DcMotor.class, "Motor1");
        rightMotor = hardwareMap.get(DcMotor.class, "Motor2");

        // Set the direction of the motors
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //open vision log file
        createVisionLogFile();
        // wait for start to be pressed
        waitForStart();
        runtime.reset();
        double prev_elapsed_time = runtime.startTime();
        String start_time_string = String.format(Locale.US,"Start Time: %.03f",prev_elapsed_time);
        telemetry.addData("Start Time",start_time_string);

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower = 0.0;
            double rightPower = 0.0;

            while (opModeIsActive()) {
                // Send calculated power to wheels
                // DJH Commenting these out for now as we just want to see what the calculated values are
                //leftMotor.setPower(leftPower);
                //rightMotor.setPower(rightPower);
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        for (Recognition recognition : updatedRecognitions) {
                            float object_height = recognition.getHeight();
                            String object_name = recognition.getLabel();
                            float object_confidence = recognition.getConfidence();
                            int object_image_height = recognition.getImageHeight();
                            double object_angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                            float object_height_ratio = object_height / object_image_height;
                            String heading;
                            RobotLog.d(
                                    String.format(Locale.US, "DJH - Detected Obj: %s Angle: %.03f Object/Image Height Ratio: %.05f",
                                            object_name,
                                            object_angle,
                                            object_height_ratio));
                            leftPower = (object_angle/45) * .25;
                            rightPower = (object_angle/45) * -.25;
                            //A negative angle means that object is to the left of the center point.
                            if (object_angle < 0) {
                                heading = "left";
                            } else {
                                heading = "right";
                            }
                            if (object_height_ratio < (.8 - .05)) {
                                //not close enough yet
                                if (Math.abs(leftPower) + Math.abs(rightPower) < .2) {
                                    //not need to turn...just go straight
                                    leftPower = .035 + (.5*((.8 - .05) - object_height_ratio));
                                    rightPower = leftPower;
                                    //set heading as straight given power levels are so similar
                                    heading = "straight - adjusted";
                                }
                            }
                            addVisionLogEntry(
                                    recognition,
                                    object_height_ratio,
                                    leftPower,
                                    rightPower,
                                    heading);
                            RobotLog.d(
                                    String.format(Locale.US, "DJH - Calculated Power => left=%.03f , right= %.03f",
                                            leftPower,
                                            rightPower
                                    ));
                        }
                        telemetry.update();
                    }
                }
                // Shows the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.update();
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }

//    private double calculateMotorPower(double current_right_power,
//                               double current_left_power,
//                               double object_angle) {
//        double new_left_power;
//        double new_right_power;
//        RobotLog.d(String.format(Locale.US,
//                "Adjusting Power - Angle (%.03f) left pwr (%.03f) right pwr (%.03f)",
//                object_angle,
//                current_left_power,
//                current_right_power));
//        new_left_power = current_left_power * object_angle;
//        new_right_power = current_right_power * object_angle;
//        return new_left_power;
//    }
    private void createVisionLogFile() {
        String visionLogFileTimeString = visionLogFileTimestamp.format(new Date());
        String visionLogFileFullName = visionLogFileBaseName + "_" + visionLogFileTimeString + ".log";
        RobotLog.d(
                String.format(Locale.US, "DJH - Opening vision log file %s",
                        visionLogFileFullName
                ));
        try
        {
            visionLog = new PrintStream(new File(visionLogFileFullName));
            visionLog.println(visionLogHeader);
            RobotLog.d(
                    String.format(Locale.US, "DJH - Vision log file %s opened successfully",
                            visionLogFileFullName
                    ));
        }
        catch (FileNotFoundException e)
        {
            visionLog = null;
        }
    }

    public static String getCurrentTimeMillis() {
        // for writing date/time to vision log file
        return visionLogDateTimeFormat.format(new Date());
    }

    public void addVisionLogEntry(Recognition recognition_object,
                                  float object_height_ratio,
                                  double leftPower,
                                  double rightPower,
                                  String heading) {
        String time_string = getCurrentTimeMillis();
        String object_name = recognition_object.getLabel();

        String object_height = String.format(Locale.US,
                "%.05f",
                recognition_object.getHeight());
        String object_image_height = String.format(Locale.US,
                "%d",
                recognition_object.getImageHeight());
        String object_width = String.format(Locale.US,
                "%.05f",
                recognition_object.getWidth());
        String object_image_width = String.format(Locale.US,
                "%d",
                recognition_object.getImageWidth());
        String object_confidence = String.format(Locale.US,
                "%.05f",
                recognition_object.getConfidence());
        String object_angle = String.format(Locale.US,
                "%f",
                recognition_object.estimateAngleToObject(AngleUnit.DEGREES));
        String left_power = String.format(Locale.US,
                "%f",
                leftPower);
        String right_power = String.format(Locale.US,
                "%f",
                rightPower);
        String object_left = String.format(Locale.US,
                "%.05f",
                recognition_object.getLeft());
        String object_right = String.format(Locale.US,
                "%.05f",
                recognition_object.getRight());
        String object_top = String.format(Locale.US,
                "%.05f",
                recognition_object.getTop());
        String object_bottom = String.format(Locale.US,
                "%.05f",
                recognition_object.getBottom());

        List<String> row_output_fields = Arrays.asList(
                time_string,
                object_name,
                object_image_height,
                object_height,
                object_image_width,
                object_width,
                object_angle,
                object_left,
                object_right,
                object_top,
                object_bottom,
                object_confidence,
                left_power,
                right_power,
                heading);
        String row_output_string = TextUtils.join("\t",row_output_fields);
        visionLog.println(row_output_string);
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        telemetry.addData("Vuforia/camera Status", "Initialized");
        telemetry.update();

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}


