package org.firstinspires.ftc.teamcode.error404;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

/**
 * An autonomous OpMode that parks the robot by strafing left by two tiles (48 inches).
 * This class extends AutoOpBase to reuse common driving and initialization logic.
 */
@Autonomous(name = "Auto Park Left (2 Tiles)")
public class AutoParkLeft extends AutoOpBase {

    /**
     * This OpMode does not use AprilTags, so we return an invalid ID.
     */
    @Override
    protected int getDesiredTagId() {
        return -1;
    }

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();

        telemetry.addLine("Autonomous Sequence:");
        telemetry.addLine("  1. Strafe Left 48\"");
        telemetry.addLine();
        telemetry.addLine("✓ Ready - Press ▶ to start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // ==================== AUTONOMOUS SEQUENCE ====================

            // Step 1: Strafe left 48 inches (2 tiles)
            currentStatus = "Parking: Strafe Left 48\"";
            updateStatusDisplay();
            driveDistance(0, -48, DRIVE_SPEED, 8.0); // Negative strafe is left

            // Sequence complete
            currentStatus = "✓ SEQUENCE COMPLETE";
            updateStatusDisplay();
            sleep(2000);
        }
    }

    /**
     * Initializes the hardware components required for this OpMode.
     * This includes the drive motors and the IMU.
     */
    private void initHardware() {
        fl = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        fr = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        bl = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        br = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust these parameters to match your robot's Control Hub orientation
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        imu.resetYaw();

        lastFL = fl.getCurrentPosition();
        lastFR = fr.getCurrentPosition();
        lastBL = bl.getCurrentPosition();
        lastBR = br.getCurrentPosition();
    }
}