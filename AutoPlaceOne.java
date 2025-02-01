package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DriveMotors;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "ScoreOneSpecimen", preselectTeleOp="Field Centric (main)")
public class AutoPlaceOne extends Auto {
	/**
	 * This function is executed when this Op Mode is initialized from the Driver Station.
	 */
	@Override
	public void runOpMode() {
		Initialize();
		MotorSetup();

		waitForStart();

		if (opModeIsActive()) {
			// Raise the arm to the high bar height
			arm.Move(BotConfig.BAR_HEIGHT, false);
			// Move the wrist to place the specimen
			intake.MoveWrist(900, false);
			// Drive towards the high bar
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * 0.9 ));
			// Wait half a second before moving arm downwards
			sleep(500);
			arm.Move(0, false);
			// Wait 2 seconds to end so the arm can go all the way down
			sleep(2000);
		}
		saveHeading();
	}
}

