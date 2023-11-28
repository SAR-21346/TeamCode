
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Auto: SkyStone Detector")
public class ColorDetecMatrix extends LinearOpMode {
    // Handle hardware stuff...

    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    ColorDetecPipeline detector = new ColorDetecPipeline(width);
    OpenCvCamera camera;

    @Override
    public void runOpMode() {
        // robot logic...

        // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        // Connect to the camera
        camera.openCameraDevice();
        // Use the ColorDetecPipeline pipeline
        // processFrame() will be called to process the frame
        camera.setPipeline(detector);
        // setup stream
        camera.startStreaming(width, height);

        //...

        ColorDetecPipeline.TeamElementLoc location = detector.getLocation();
        if (location == ColorDetecPipeline.TeamElementLoc.NONE) {
            telemetry.addLine("Element is Center");
        }
        else if (location == ColorDetecPipeline.TeamElementLoc.LEFT) {
            telemetry.addLine("Element is Left");
        }
        else if (location == ColorDetecPipeline.TeamElementLoc.RIGHT) {
            telemetry.addLine("Element is Right");
    }

}
