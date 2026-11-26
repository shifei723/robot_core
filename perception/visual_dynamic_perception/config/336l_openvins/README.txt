Gemini 336L CPC87630003A: stereo infrared (640x400@30Hz) + IMU (200Hz).
Camera intrinsics/extrinsics were captured from this device's factory CameraInfo/TF.
T_cam_imu = inverse(body_T_camera) * body_T_imu (Kalibr/OpenVINS convention).
Noise densities/random walks are engineering initial values, not measured Allan results.
Camera/IMU temporal offset starts at zero; independent temporal calibration remains necessary for accuracy assessment.
Calibration optimization disabled until sufficient motion excitation and calibrated data are available.
Conditional ZUPT enabled using OpenVINS visual disparity check (chi2 multiplier 0, disparity 0.2px).
IMU remains active in initialization, propagation and fusion; this is not a fixed-camera mode.
Live startup regenerates these files under runtime/336l/openvins using the selected camera.
