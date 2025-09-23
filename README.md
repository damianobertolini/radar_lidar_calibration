Task: Make a tool to perform the extrinsic calibration of the radar and lidar ( rotation and translation matrix )

The radar has no elevation so z component is not reliable and should be ignored, that makes the problem simpler as we need just one angle in the rotation matrix.

In the test data you have the left front camera, lidar and front left radar (1jpeg, 2csv files). Data with same number is from almost same timestamp.  The camera is not needed for the calibration, it is just for reference reprojection. Basically , what you have are a set of measurements with a corner reflector and the lidar, you could match them and get the task done. We use the corner reflect as that reflection is strong and we ca see the point and also we see the corner reflector in the lidar.

screenshot.jpg shows a top view of the lidar and radar points projected on the ground ( blue lidar, red radar ) given some initial guess of transformation.

The lidar and radar csv files can be read with : 

    df = pd.read_csv(path)
    points = df[['x', 'y', 'z']].to_numpy()

You can use as initial guess the translation : 

    array([ 2.856,  0.635, -1.524]) 

and a rotation matrix (if helpful):

        th = np.deg2rad(45)
        R = np.array([[np.cos(th), -np.sin(th), 0.0],
                        [np.sin(th),  np.cos(th), 0.0],
                        [0.0,         0.0,        1.0]])

You can reproject on camera and see if results are ok. To read the calibration files use :

    Camera intrinsics like in open cv (https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) 

    calib = np.load(calib_cam_npz_file_repo)
    K = calib["camera_matrix"]
    dist_coeffs = calib["dist_coeffs"]

    and lidar to camera calibration :

    calib = np.load(calib_lidar2cam_file_npz_repo)
    t_lidar_cam = calib["t"]
    R_lidar_cam = calib["R"]


USE AI to not spend ages on it ! Use whatever you like to code and send us your github link. Write 3-4 lines to explain what you have done.


