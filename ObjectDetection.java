package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config;
import java.util.List;



public class ObjectDetection {
	private LinearOpMode auto;
	private TfodProcessor tfod;
	private VisionPortal visionPortal;


	public ObjectDetection(LinearOpMode auto) {
		this.auto = auto;

		// Create the TensorFlow processor by using a builder.
		tfod = new TfodProcessor.Builder()
			.setModelFileName(Config.TFOD_MODEL_FILE)
			.setModelLabels(Config.LABELS)
			.build();

		// Create the vision portal by using a builder.
		VisionPortal.Builder builder = new VisionPortal.Builder();

		// Set the camera (webcam vs. built-in RC phone camera).
		builder.setCamera(auto.hardwareMap.get(CameraName.class, Config.CAMERA_NAME));

		// Set and enable the processor.
		builder.addProcessor(tfod);

		// Build the Vision Portal, using the above settings.
		visionPortal = builder.build();
	}


	public int GetPropPos() { // 0 = left, 1 = center, 2 = right
		Recognition recognition = highestConfidence();

		if (recognition != null) {
			double propX = (recognition.getLeft() + recognition.getRight()) / 2 ;

			if ((Config.CAMERA_RESO_X / 2) > propX) {
				return 1;
			}
			return 2;
		}
		return 0;
	}


	private Recognition highestConfidence() {
		Recognition recognition = null;

		for (int i = 0; i <= 1000; i++) {
			List<Recognition> currentRecognitions = tfod.getRecognitions();

			if (currentRecognitions.size() < 1) {
				auto.sleep(10);
				continue;
			}
			
			// Select highest confidence object
			for (Recognition potentialRecognition : currentRecognitions) {

				if (recognition != null && recognition.getConfidence() > potentialRecognition.getConfidence()) {
					continue;
				}
				recognition = potentialRecognition;
			}
			return recognition;
		}
		return recognition;
	}
}
