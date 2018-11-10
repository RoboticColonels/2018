package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.DcMotorSimple

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector

internal class MecanumDrive(hardwareMap: HardwareMap) {
    private val frontLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "front_left")
    private val frontRight: DcMotor = hardwareMap.get(DcMotor::class.java, "front_right")
    private val rearLeft: DcMotor = hardwareMap.get(DcMotor::class.java, "rear_left")
    private val rearRight: DcMotor = hardwareMap.get(DcMotor::class.java, "rear_right")
    private val lift: DcMotor = hardwareMap.get(DcMotor::class.java, "lift")
    private val latch: Servo = hardwareMap.get(Servo::class.java, "latche_servo")
    private val dump: Servo = hardwareMap.get(Servo::class.java, "dump_servo")

    init {
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        rearRight.direction = DcMotorSimple.Direction.REVERSE
    }

    fun drive(x: Double, y: Double, r: Double) {
        frontLeft.power = x + y + r
        frontRight.power = x - y - r
        rearLeft.power = x - y + r
        rearRight.power = x + y - r
    }

    fun setLift(power: Double) {
        lift.power = power
    }

    fun setLatch(position: Double) {
        latch.position = position
    }

    fun setDump(position: Double) {
        dump.position = position
    }
}

@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
internal class MecanumTeleOp : LinearOpMode() {
    override fun runOpMode() {
        val driver = MecanumDrive(hardwareMap)

        waitForStart()

        var latchPosition = 0.0

        while (opModeIsActive()) {
            val x = -gamepad1.left_stick_y.toDouble()
            val y = gamepad1.left_stick_x.toDouble()
            val r = -gamepad1.right_stick_x.toDouble()
            driver.drive(x, y, r)

            driver.setLift(gamepad2.left_stick_y.toDouble())

            if (gamepad2.right_bumper && latchPosition < 1.0) {
                latchPosition += 0.05
            } else if (gamepad2.left_bumper && latchPosition > -1.0) {
                latchPosition -= 0.05
            }
            driver.setLatch(latchPosition)
        }
    }
}

@Autonomous(name = "Mecanum Autonomous", group = "Autonomous")
internal class MecanumAutonomus : LinearOpMode() {
    override fun runOpMode() {
        val driver = MecanumDrive(hardwareMap)

        // runRecognition()
        telemetry.update()

        waitForStart()

        driver.setLift(1.0)

        sleep(1500)

        driver.setLift(0.0)

        driver.setLatch(0.0)

        sleep(4000)

        driver.drive(-0.5, 0.0, 0.0)

        sleep(2000)

        driver.drive(0.0, 0.0, 0.0)

        driver.setLift(-1.0)

        driver.setDump(1.0)

        sleep(1000)

        driver.setLift(0.0)

        driver.setLatch(0.0)
    }
}