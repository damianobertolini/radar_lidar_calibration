import os
import re
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree
from scipy.optimize import minimize
import cv2
import matplotlib.pyplot as plt
from PIL import Image


def load_points_from_csv(path):
    """Load 3D points from CSV file."""
    df = pd.read_csv(path)
    points = df[['x', 'y', 'z']].to_numpy()
    # filter within 10m radius
    mask = np.sum(points**2, axis=1) <= 100
    return points[mask]


def find_corner_reflector_correspondence(lidar_file, radar_file, script_dir, th, t_ref, z_min, z_max, r_gate):
    """
    Find corner reflector correspondence for a single file pair.
    
    Args:
        lidar_file: Path to lidar CSV file
        radar_file: Path to radar CSV file
        script_dir: Script directory for saving files
        th: Initial rotation angle
        t_ref: Initial translation vector
        z_min, z_max: Z coordinate bounds for filtering
        r_gate: Radius for neighbor search
    
    Returns:
        lidar_centroid: Centroid of matched lidar points (or None if not found)
        radar_point: Selected radar point (or None if not found)
        lidar_filter: Matched lidar points (or None if not found)
    """
    # Load points
    lidar_points = load_points_from_csv(lidar_file)
    radar_points_raw = load_points_from_csv(radar_file)
    
    # Transform radar points to lidar frame using initial guess
    radar_transformed = radar_to_lidar_transform(radar_points_raw, th, t_ref)
    
    # Filter lidar points by z and range
    rL = np.linalg.norm(lidar_points[:, :2], axis=1)
    mask0 = (lidar_points[:, 2] > z_min) & (lidar_points[:, 2] < z_max) & (rL > 2.0) & (rL < 30.0)
    lidar_filtered = lidar_points[mask0]
    
    print(f"  Filtered lidar points: {len(lidar_points)} -> {len(lidar_filtered)}")
    
    if len(lidar_filtered) == 0:
        print(f"  No lidar points after filtering")
        return None, None, None
    
    # Build KD-Tree for lidar points
    tree = cKDTree(lidar_filtered[:, :2])
    
    # Find best radar point
    lidar_filter = []
    radar_idx = 0
    lidar_count = 0
    
    for i in range(radar_transformed.shape[0]):
        r = radar_transformed[i]
        idx_local = tree.query_ball_point(r[:2], r=r_gate)
        if idx_local:
            lidar_filter.append(lidar_filtered[idx_local])
            count = len(idx_local)
            # select the radar point with most nearby lidar points
            if count > lidar_count:
                lidar_count = count
                radar_idx = i
    
    if len(lidar_filter) > 0:
        # Stack all matched lidar points
        lidar_filter = np.vstack(lidar_filter)
        
        # Get the selected radar point
        selected_radar = radar_transformed[radar_idx:radar_idx+1]
        original_radar = radar_points_raw[radar_idx:radar_idx+1]
        
        # Compute centroid of matched lidar points
        lidar_centroid = np.mean(lidar_filter, axis=0)
        
        print(f"  Found corner reflector: {lidar_count} nearby lidar points")
        print(f"  Lidar centroid: {lidar_centroid}")
        print(f"  Radar point: {selected_radar[0]}")
        
        # Save filtered points
        path_name = os.path.basename(lidar_file).replace(".csv", "")
        np.save(os.path.join(script_dir, f'point_matches_reflector/lidar_points_{path_name}.npy'), lidar_filter)
        np.save(os.path.join(script_dir, f'point_matches_reflector/radar_points_{path_name}.npy'), original_radar)
        
        return lidar_centroid, selected_radar[0], lidar_filter
    else:
        print(f"  No corner reflector found")
        return None, None, None


def radar_to_lidar_transform(points_radar, angle, translation):
    """
    Transform radar points to lidar frame using rotation angle and translation.
    
    Args:
        points_radar: Nx3 array of radar points
        angle: rotation angle in radians
        translation: 3D translation vector
    
    Returns:
        Transformed points in lidar frame
    """
    # Create rotation matrix (only around z-axis since radar has no elevation)
    R = np.array([[np.cos(angle), -np.sin(angle), 0.0],
                  [np.sin(angle),  np.cos(angle), 0.0],
                  [0.0,            0.0,           1.0]])
    
    # Apply transformation
    return (R @ points_radar.T).T + translation


def calibration_error(params, radar_points, lidar_points):
    """
    Compute calibration error between radar and lidar points.
    
    Args:
        params: [angle, tx, ty, tz] - rotation angle and translation
        radar_points: Nx3 array of radar points
        lidar_points: Nx3 array of corresponding lidar points
    
    Returns:
        Mean squared error
    """
    angle, tx, ty, tz = params
    translation = np.array([tx, ty, tz])
    
    # Transform radar points to lidar frame
    transformed_radar = radar_to_lidar_transform(radar_points, angle, translation)
    
    # Compute mean squared error
    if len(transformed_radar) == 0 or len(lidar_points) == 0:
        return float('inf')
    
    # For each transformed radar point, find closest lidar point
    lidar_tree = cKDTree(lidar_points)
    distances, _ = lidar_tree.query(transformed_radar)
    
    return np.mean(distances**2)


def solve_se2_kabsch(P_r_xy, P_l_xy, w=None):
    """
    Solve 2D rigid transformation using Kabsch algorithm (SVD).
    
    Args:
        P_r_xy: Radar points (N,2)
        P_l_xy: Lidar points (N,2)
        w: Weights (optional)
    
    Returns:
        theta: rotation angle in radians
        R2: 2x2 rotation matrix
        txy: 2D translation vector
    """
    Pr = np.asarray(P_r_xy)
    Pl = np.asarray(P_l_xy)
    if w is None: 
        w = np.ones(len(Pr))
    w = w / (w.sum() + 1e-9)
    
    # Compute centroids
    mr = (w[:, None] * Pr).sum(0)
    ml = (w[:, None] * Pl).sum(0)
    
    # Center the points
    Qr = (Pr - mr) * w[:, None]
    Ql = (Pl - ml)
    
    # Compute cross-covariance matrix
    H = Qr.T @ Ql  # 2x2
    
    # SVD decomposition
    U, _, Vt = np.linalg.svd(H)
    R2 = Vt.T @ U.T
    
    # Ensure proper rotation (det(R) = 1)
    if np.linalg.det(R2) < 0:
        Vt[1, :] *= -1
        R2 = Vt.T @ U.T
    
    # Extract rotation angle
    theta = np.arctan2(R2[1, 0], R2[0, 0])
    
    # Compute translation
    txy = ml - R2 @ mr
    
    return theta, R2, txy


def solve_se2_icp(P_r_xy, P_l_xy, max_iterations=50, tolerance=1e-6):
    """
    Solve 2D rigid transformation using ICP (Iterative Closest Point).
    
    Args:
        P_r_xy: Radar points (N,2)
        P_l_xy: Lidar points (N,2)
        max_iterations: Maximum number of iterations
        tolerance: Convergence tolerance
    
    Returns:
        theta: rotation angle in radians
        R2: 2x2 rotation matrix
        txy: 2D translation vector
    """
    Pr = np.asarray(P_r_xy)
    Pl = np.asarray(P_l_xy)
    
    # Initialize transformation
    R2 = np.eye(2)
    txy = np.zeros(2)
    
    # Build KD-tree for lidar points
    lidar_tree = cKDTree(Pl)
    
    prev_error = float('inf')
    
    for iteration in range(max_iterations):
        # Transform radar points
        Pr_transformed = (R2 @ Pr.T).T + txy
        
        # Find closest lidar points
        distances, indices = lidar_tree.query(Pr_transformed)
        
        # Get corresponding lidar points
        Pl_correspondences = Pl[indices]
        
        # Compute current error
        current_error = np.mean(distances**2)
        
        # Check convergence
        if abs(prev_error - current_error) < tolerance:
            print(f"ICP converged after {iteration + 1} iterations")
            break
            
        prev_error = current_error
        
        # Solve for new transformation using Kabsch on correspondences
        theta_new, R2_new, txy_new = solve_se2_kabsch(Pr, Pl_correspondences)
        
        # Update transformation
        R2 = R2_new @ R2
        txy = R2_new @ txy + txy_new
    
    # Extract final rotation angle
    theta = np.arctan2(R2[1, 0], R2[0, 0])
    
    return theta, R2, txy


def solve_se2_ransac(P_r_xy, P_l_xy, max_iterations=1000, threshold=0.5, min_inliers=3):
    """
    Solve 2D rigid transformation using RANSAC + Kabsch.
    
    Args:
        P_r_xy: Radar points (N,2)
        P_l_xy: Lidar points (N,2)
        max_iterations: Maximum RANSAC iterations
        threshold: Inlier threshold
        min_inliers: Minimum number of inliers
    
    Returns:
        theta: rotation angle in radians
        R2: 2x2 rotation matrix
        txy: 2D translation vector
    """
    Pr = np.asarray(P_r_xy)
    Pl = np.asarray(P_l_xy)
    
    if len(Pr) < min_inliers:
        return solve_se2_kabsch(Pr, Pl)
    
    best_inliers = 0
    best_R2 = np.eye(2)
    best_txy = np.zeros(2)
    
    for _ in range(max_iterations):
        # Randomly sample minimum number of points
        sample_indices = np.random.choice(len(Pr), min_inliers, replace=False)
        Pr_sample = Pr[sample_indices]
        Pl_sample = Pl[sample_indices]
        
        # Solve transformation for this sample
        try:
            theta_sample, R2_sample, txy_sample = solve_se2_kabsch(Pr_sample, Pl_sample)
            
            # Transform all radar points
            Pr_transformed = (R2_sample @ Pr.T).T + txy_sample
            
            # Count inliers
            distances = np.linalg.norm(Pr_transformed - Pl, axis=1)
            inliers = np.sum(distances < threshold)
            
            if inliers > best_inliers:
                best_inliers = inliers
                best_R2 = R2_sample
                best_txy = txy_sample
                
        except:
            continue
    
    # Refine with all inliers
    if best_inliers >= min_inliers:
        Pr_transformed = (best_R2 @ Pr.T).T + best_txy
        distances = np.linalg.norm(Pr_transformed - Pl, axis=1)
        inlier_mask = distances < threshold
        
        if np.sum(inlier_mask) >= min_inliers:
            theta_final, R2_final, txy_final = solve_se2_kabsch(Pr[inlier_mask], Pl[inlier_mask])
            return theta_final, R2_final, txy_final
    
    # Fallback to Kabsch if RANSAC fails
    return solve_se2_kabsch(Pr, Pl)


def solve_calibration(test_data_folder, initial_angle_deg=50, initial_translation=None, algorithm='kabsch'):
    """
    Solve radar-lidar calibration using different algorithms.
    
    Args:
        test_data_folder: Path to folder containing test data
        initial_angle_deg: Initial guess for rotation angle in degrees
        initial_translation: Initial guess for translation vector
        algorithm: 'kabsch', 'icp', or 'ransac'
    
    Returns:
        best_angle: Optimal rotation angle in radians
        best_translation: Optimal translation vector
        correspondences: List of (radar_point, lidar_points) correspondences
    """
    
    # Get file pairs
    files = [f for f in os.listdir(test_data_folder) if f.endswith(".csv")]
    files.sort(key=lambda x: int(re.findall(r'\d+', x)[-1]) if re.findall(r'\d+', x) else 0)
    
    pairs = []
    for f in files:
        if "_rad" in f.lower():
            continue
        radar_candidate = f.replace(".csv", "_rad.csv")
        if radar_candidate in files:
            pairs.append((os.path.join(test_data_folder, f), 
                         os.path.join(test_data_folder, radar_candidate)))
    
    print(f"Found {len(pairs)} lidar-radar file pairs")

    # initial rotation and translation based on suggestion in the task
    th = np.deg2rad(initial_angle_deg)
    R_ref = np.array([[np.cos(th), -np.sin(th), 0.0],
                      [np.sin(th),  np.cos(th), 0.0],
                      [0.0,         0.0,        1.0]])
    t_ref = initial_translation # can both use get_matrices_init in quick_view.py or the suggested initial translation

    # Create output directories
    os.makedirs(os.path.join(script_dir, "point_matches_reflector"), exist_ok=True)
    os.makedirs(os.path.join(script_dir, "calibration_output"), exist_ok=True)
    
    # Collect correspondences from all files
    lidar_centroids = []
    radar_points = []
    correspondences = []
    
    z_min, z_max = -1.0, 2.5
    r_gate = 1.0
    
    for lidar_file, radar_file in pairs:
        print(f"Processing {os.path.basename(lidar_file)} and {os.path.basename(radar_file)}")
        
        # Find corner reflector correspondence
        lidar_centroid, radar_point, lidar_filter = find_corner_reflector_correspondence(
            lidar_file, radar_file, script_dir, th, t_ref, z_min, z_max, r_gate
        )
        
        if lidar_centroid is not None and radar_point is not None and lidar_filter is not None:
            lidar_centroids.append(lidar_centroid)
            radar_points.append(radar_point)
            correspondences.append((radar_point, lidar_filter))
    
    if len(lidar_centroids) == 0:
        print("No correspondences found!")
        return None, None, []
    
    lidar_centroids = np.array(lidar_centroids)
    radar_points = np.array(radar_points)
    
    print(f"Total correspondences: {len(lidar_centroids)}")
    
    # Solve 2D rigid transformation using selected algorithm
    print(f"Solving 2D rigid transformation using {algorithm.upper()} algorithm...")
    
    if algorithm == 'kabsch':
        theta, R2, txy = solve_se2_kabsch(radar_points[:, :2], lidar_centroids[:, :2])
    elif algorithm == 'icp':
        theta, R2, txy = solve_se2_icp(radar_points[:, :2], lidar_centroids[:, :2])
    elif algorithm == 'ransac':
        theta, R2, txy = solve_se2_ransac(radar_points[:, :2], lidar_centroids[:, :2])
    else:
        print(f"Unknown algorithm: {algorithm}. Using Kabsch.")
        theta, R2, txy = solve_se2_kabsch(radar_points[:, :2], lidar_centroids[:, :2])
    
    # Extend to 3D
    R = np.eye(3)
    R[:2, :2] = R2
    t = np.array([txy[0], txy[1], np.median(lidar_centroids[:, 2])])
    
    # Compose with initial transformation
    R_final = R @ R_ref
    t_final = t + R @ t_ref
    
    # Extract final angle
    final_angle = np.arctan2(R_final[1, 0], R_final[0, 0])
    
    print(f"Calibration successful!")
    print(f"Final angle: {np.rad2deg(final_angle):.2f} degrees")
    print(f"Final translation: {t_final}")
    
    # Save the final calibration
    calibration_result = {
        "R": R_final,
        "t": t_final
    }
    np.savez(os.path.join(script_dir, "calibration_output/radar_lidar_calibration.npz"), **calibration_result)
    
    return final_angle, t_final, correspondences


def project_to_camera_and_visualize_all_images(calib_cam_file, calib_lidar2cam_file, test_data_folder):
    """
    Project radar and lidar points to camera for all test images.
    
    Args:
        calib_cam_file: path to camera calibration file
        calib_lidar2cam_file: path to lidar-to-camera calibration file
        test_data_folder: path to test data folder
    """
    # Load camera calibration
    calib_cam = np.load(calib_cam_file)
    K = calib_cam["camera_matrix"]
    dist_coeffs = calib_cam["dist_coeffs"]
    
    # Load lidar-to-camera calibration
    calib_lidar2cam = np.load(calib_lidar2cam_file)
    t_lidar_cam = calib_lidar2cam["t"]
    R_lidar_cam = calib_lidar2cam["R"]
    
    # Load radar-to-lidar calibration
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calibration_result = np.load(os.path.join(script_dir, "calibration_output/radar_lidar_calibration.npz"))
    R_radar_lidar = calibration_result["R"]
    t_radar_lidar = calibration_result["t"]
    
    def project_to_image(points_cam, K, dist_coeffs):
        """Project 3D points in camera frame onto image plane."""
        proj_points, _ = cv2.projectPoints(
            points_cam, np.zeros((3, 1)), np.zeros((3, 1)), K, dist_coeffs
        )
        return proj_points.reshape(-1, 2)
    
    # Process all test images
    file_list = [8, 16, 24, 31, 64]
    
    for f_id in file_list:
        print(f"Processing image {f_id}.jpg")
        
        # Load image
        image_path = os.path.join(test_data_folder, f'{f_id}.jpg')
        if not os.path.exists(image_path):
            print(f"  Image {f_id}.jpg not found, skipping")
            continue
            
        image = np.array(Image.open(image_path))
        
        # Load filtered points
        lidar_path = os.path.join(script_dir, f'point_matches_reflector/lidar_points_{f_id}.npy')
        radar_path = os.path.join(script_dir, f'point_matches_reflector/radar_points_{f_id}.npy')
        
        if os.path.exists(lidar_path) and os.path.exists(radar_path):
            lidar_filter = np.load(lidar_path)
            radar_filter = np.load(radar_path)
            
            # Get z value from lidar points
            z = np.mean(lidar_filter, axis=0)[2]
            
            # Transform radar points to lidar frame using final calibration
            radar_points = radar_to_lidar_transform(radar_filter, 
                                                   np.arctan2(R_radar_lidar[1, 0], R_radar_lidar[0, 0]), 
                                                   t_radar_lidar)
            # Set z value from lidar
            radar_points[:, 2] = z
            
            # Transform to camera frame
            lidar_points_cam = (R_lidar_cam @ lidar_filter.T).T + t_lidar_cam
            radar_points_cam = (R_lidar_cam @ radar_points.T).T + t_lidar_cam
            
            # Project to image
            lidar_proj = project_to_image(lidar_points_cam, K, dist_coeffs)
            radar_proj = project_to_image(radar_points_cam, K, dist_coeffs)
            
            # Filter points within image bounds
            x_lidar, y_lidar = lidar_proj[:, 0], lidar_proj[:, 1]
            x_radar, y_radar = radar_proj[:, 0], radar_proj[:, 1]
            
            mask_lidar = (x_lidar >= 0) & (x_lidar < image.shape[1]) & (y_lidar >= 0) & (y_lidar < image.shape[0])
            mask_radar = (x_radar >= 0) & (x_radar < image.shape[1]) & (y_radar >= 0) & (y_radar < image.shape[0])
            
            lidar_proj = lidar_proj[mask_lidar]
            radar_proj = radar_proj[mask_radar]
            
            # Create visualization
            plt.figure(figsize=(10, 8))
            plt.imshow(image)
            plt.scatter(lidar_proj[:, 0], lidar_proj[:, 1], c='blue', s=5, label='Lidar', alpha=0.7)
            plt.scatter(radar_proj[:, 0], radar_proj[:, 1], c='red', s=20, marker='x', linewidth=2, label='Radar')
            plt.title(f'Calibration Result - Image {f_id}')
            plt.legend()
            plt.axis('off')
            plt.tight_layout()
            plt.savefig(os.path.join(script_dir, f'calibration_output/radar_lidar_projection_{f_id}.png'), dpi=150, bbox_inches='tight')
            plt.close()
            
            print(f"  Saved radar_lidar_projection_{f_id}.png")
        else:
            print(f"  Filtered points not found for {f_id}, skipping")
    
    print("All calibration images saved!")


def visualize_calibration_results_from_files():
    """
    Visualize calibration results using the saved filtered points.
    """
    plt.figure(figsize=(12, 10))
    
    # Load calibration results
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calibration_result = np.load(os.path.join(script_dir, "calibration_output/radar_lidar_calibration.npz"))
    R_radar_lidar = calibration_result["R"]
    t_radar_lidar = calibration_result["t"]
    
    # Colors for different files
    colors = ['red', 'blue', 'green', 'orange', 'purple']
    
    # Get file list
    file_list = [8, 16, 24, 31, 64]
    
    for i, f_id in enumerate(file_list):
        color = colors[i % len(colors)]
        
        # Load filtered points
        lidar_path = os.path.join(script_dir, f'point_matches_reflector/lidar_points_{f_id}.npy')
        radar_path = os.path.join(script_dir, f'point_matches_reflector/radar_points_{f_id}.npy')
        
        if os.path.exists(lidar_path) and os.path.exists(radar_path):
            lidar_filter = np.load(lidar_path)
            radar_filter = np.load(radar_path)
            
            # Transform radar point to lidar frame using final calibration
            radar_transformed = radar_to_lidar_transform(radar_filter, 
                                                       np.arctan2(R_radar_lidar[1, 0], R_radar_lidar[0, 0]), 
                                                       t_radar_lidar)
            
            # Plot lidar points (blue)
            plt.scatter(lidar_filter[:, 0], lidar_filter[:, 1], 
                       c='blue', s=1, alpha=0.6, label='Lidar' if i == 0 else "")
            
            # Plot transformed radar point (colored X)
            plt.scatter(radar_transformed[0, 0], radar_transformed[0, 1], 
                       c=color, s=50, marker='x', linewidth=3, 
                       label=f'Radar {f_id}' if i < 5 else "")
            
            # Plot original radar point (before transformation) in gray
            plt.scatter(radar_filter[0, 0], radar_filter[0, 1], 
                       c='gray', s=30, marker='o', alpha=0.5)
    
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Calibration Results - Top View\nBlue: Filtered Lidar points, Colored X: Transformed radar points, Gray O: Original radar points')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    plt.tight_layout()
    plt.savefig(os.path.join(script_dir, 'calibration_output/radar_lidar_calibration_overview.png'), dpi=150, bbox_inches='tight')
    plt.close()
    
    print("Calibration visualization saved as 'calibration_output/radar_lidar_calibration_overview.png'")


if __name__ == "__main__":
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Configuration - all paths relative to script directory
    test_data_folder = os.path.join(script_dir, "test_data")
    calib_cam_file = os.path.join(script_dir, "cfl_calibration.npz")
    calib_lidar2cam_file = os.path.join(script_dir, "lidar2cfl_new_all.npz")
    
    # Initial guess from README
    initial_angle_deg = 50
    initial_translation = np.array([2.856, 0.635, -1.524])
    
    print("Starting radar-lidar calibration...")
    print("=" * 50)
    
    # Get file pairs for later use
    files = [f for f in os.listdir(test_data_folder) if f.endswith(".csv")]
    files.sort(key=lambda x: int(re.findall(r'\d+', x)[-1]) if re.findall(r'\d+', x) else 0)
    
    pairs = []
    for f in files:
        if "_rad" in f.lower():
            continue
        radar_candidate = f.replace(".csv", "_rad.csv")
        if radar_candidate in files:
            pairs.append((os.path.join(test_data_folder, f), 
                         os.path.join(test_data_folder, radar_candidate)))
    
    # Solve calibration using Kabsch algorithm (best for this task)
    print("Using KABSCH algorithm for calibration...")
    best_angle, best_translation, correspondences = solve_calibration(
        test_data_folder, initial_angle_deg, initial_translation, algorithm='kabsch'
    )
    
    # Note: Other algorithms (ICP, RANSAC) are available in the code above
    # To test them, uncomment the following section:
    """
    # Test different algorithms for comparison
    algorithms = ['icp', 'ransac']
    for algo in algorithms:
        print(f"\nTesting {algo.upper()} algorithm:")
        angle, translation, _ = solve_calibration(
            test_data_folder, initial_angle_deg, initial_translation, algorithm=algo
        )
        if angle is not None:
            print(f"{algo.upper()} Results: {np.rad2deg(angle):.2f}°, {translation}")
    """
    
    if best_angle is not None:
        print("\n" + "=" * 50)
        print("CALIBRATION RESULTS:")
        print(f"Rotation angle: {np.rad2deg(best_angle):.2f} degrees")
        print(f"Translation: [{best_translation[0]:.3f}, {best_translation[1]:.3f}, {best_translation[2]:.3f}]")
        print(f"Number of correspondences used: {len(correspondences)}")
        
        # Create rotation matrix
        R = np.array([[np.cos(best_angle), -np.sin(best_angle), 0.0],
                      [np.sin(best_angle),  np.cos(best_angle), 0.0],
                      [0.0,                 0.0,                1.0]])
        
        print(f"\nRotation matrix R:")
        print(R)
        print(f"\nTranslation vector t:")
        print(best_translation)
        
        # Test projection to camera (if calibration files exist)
        if os.path.exists(calib_cam_file) and os.path.exists(calib_lidar2cam_file):
            print("\nTesting projection to camera...")
            project_to_camera_and_visualize_all_images(calib_cam_file, calib_lidar2cam_file, test_data_folder)
        
        # Visualize calibration results
        print("\nCreating calibration visualization...")
        visualize_calibration_results_from_files()
    else:
        print("Calibration failed!")
