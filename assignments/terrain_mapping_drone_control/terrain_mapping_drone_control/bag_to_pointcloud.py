#!/usr/bin/env python3
"""
Reads a ROS 2 bag file and generates a 3D point cloud using Open3D.
Fuses RGB-D frames with odometry poses with correct camera extrinsics.
"""

import numpy as np
import open3d as o3d
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation
import os

# ── Configuration ─────────────────────────────────────────────────────────────
BAG_PATH    = os.path.expanduser('~/ros2_ws/mission_bag')
OUTPUT_PATH = os.path.expanduser('~/ros2_ws/mission_map.ply')

RGB_TOPIC         = '/drone/front_rgb'
DEPTH_TOPIC       = '/drone/front_depth'
CAMERA_INFO_TOPIC = '/drone/front_rgb/camera_info'
ODOM_TOPIC        = '/drone/odom'

FRAME_SKIP = 1      # process every frame
MIN_DEPTH  = 0.5
MAX_DEPTH  = 18.0
VOXEL_SIZE = 0.05
# ──────────────────────────────────────────────────────────────────────────────

def make_transform(x, y, z, roll, pitch, yaw):
    T = np.eye(4)
    T[:3,:3] = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T

# OpenCV optical frame -> ROS body frame
# OpenCV: X right, Y down,  Z forward
# ROS:    X fwd,   Y left,  Z up
# cam_Z -> body_X, -cam_X -> body_Y, -cam_Y -> body_Z
OPTICAL_TO_BODY = np.array([
    [ 0,  0,  1,  0],
    [-1,  0,  0,  0],
    [ 0, -1,  0,  0],
    [ 0,  0,  0,  1]], dtype=float)

# Camera mount on drone: position + 30 deg downward tilt
BODY_T_CAMERA = make_transform(0.12, 0.03, 0.242, 0, 0.523, 0)

def get_world_T_optical(odom_msg):
    """Full transform: optical frame -> world frame."""
    p = odom_msg.pose.pose.position
    q = odom_msg.pose.pose.orientation
    world_T_body = np.eye(4)
    world_T_body[:3,:3] = Rotation.from_quat(
        [q.x, q.y, q.z, q.w]).as_matrix()
    world_T_body[:3, 3] = [p.x, p.y, p.z]
    return world_T_body @ BODY_T_CAMERA @ OPTICAL_TO_BODY

def get_intrinsics(cam_info_msg):
    k = cam_info_msg.k
    return o3d.camera.PinholeCameraIntrinsic(
        cam_info_msg.width, cam_info_msg.height,
        k[0], k[4], k[2], k[5])

def decode_depth(img_msg):
    enc = img_msg.encoding
    h, w = img_msg.height, img_msg.width
    if enc == '32FC1':
        d = np.frombuffer(img_msg.data, dtype=np.float32).reshape((h, w))
    elif enc == '16UC1':
        d = np.frombuffer(img_msg.data, dtype=np.uint16).reshape(
            (h, w)).astype(np.float32) / 1000.0
    else:
        raise ValueError(f'Unsupported depth encoding: {enc}')
    return np.where(
        np.isnan(d) | np.isinf(d) | (d < MIN_DEPTH) | (d > MAX_DEPTH),
        0.0, d)

def decode_rgb(img_msg):
    h, w = img_msg.height, img_msg.width
    enc  = img_msg.encoding
    data = np.frombuffer(img_msg.data, dtype=np.uint8)
    if enc == 'rgb8':
        return data.reshape((h, w, 3))
    if enc == 'bgr8':
        return data.reshape((h, w, 3))[:, :, ::-1]
    if enc == 'mono8':
        img = data.reshape((h, w))
        return np.stack([img, img, img], axis=-1)
    raise ValueError(f'Unsupported rgb encoding: {enc}')

def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

def main():
    typestore = get_typestore(Stores.ROS2_JAZZY)
    print(f'Opening bag: {BAG_PATH}')

    # ── Collect all messages ──────────────────────────────────────────────────
    rgb_msgs   = {}
    depth_msgs = {}
    odom_msgs  = {}
    cam_info   = None

    with Reader(BAG_PATH) as reader:
        for conn, timestamp, rawdata in reader.messages():
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            if conn.topic == RGB_TOPIC:
                rgb_msgs[stamp_to_sec(msg.header.stamp)] = msg
            elif conn.topic == DEPTH_TOPIC:
                depth_msgs[stamp_to_sec(msg.header.stamp)] = msg
            elif conn.topic == CAMERA_INFO_TOPIC and cam_info is None:
                cam_info = msg
            elif conn.topic == ODOM_TOPIC:
                odom_msgs[stamp_to_sec(msg.header.stamp)] = msg

    print(f'RGB: {len(rgb_msgs)}  Depth: {len(depth_msgs)}  '
          f'Odom: {len(odom_msgs)}')

    if cam_info is None:
        print('ERROR: No camera info found!')
        return

    intrinsics  = get_intrinsics(cam_info)
    rgb_times   = sorted(rgb_msgs.keys())
    depth_times = sorted(depth_msgs.keys())
    odom_times  = sorted(odom_msgs.keys())

    combined = o3d.geometry.PointCloud()
    processed = skipped = 0

    print(f'\nProcessing {len(rgb_times)} frames '
          f'(skip={FRAME_SKIP}, depth {MIN_DEPTH}-{MAX_DEPTH}m)...')

    for i, rgb_t in enumerate(rgb_times):
        if i % FRAME_SKIP != 0:
            continue

        # Match depth frame
        depth_t = min(depth_times, key=lambda t: abs(t - rgb_t))
        if abs(depth_t - rgb_t) > 0.1:
            skipped += 1
            continue

        # Match odom
        odom_t = min(odom_times, key=lambda t: abs(t - rgb_t))
        if abs(odom_t - rgb_t) > 0.5:
            skipped += 1
            continue

        try:
            rgb   = decode_rgb(rgb_msgs[rgb_t])
            depth = decode_depth(depth_msgs[depth_t])
            odom  = odom_msgs[odom_t]

            valid_px = np.sum(depth > 0)
            if valid_px < 100:
                skipped += 1
                continue

            rgb_o3d = o3d.geometry.Image(rgb.astype(np.uint8))
            dep_o3d = o3d.geometry.Image((depth * 1000.0).astype(np.uint16))

            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                rgb_o3d, dep_o3d,
                depth_scale=1000.0,
                depth_trunc=MAX_DEPTH,
                convert_rgb_to_intensity=False)

            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd, intrinsics)

            if len(pcd.points) == 0:
                skipped += 1
                continue

            pcd.transform(get_world_T_optical(odom))
            combined += pcd
            processed += 1

            if processed % 20 == 0:
                z = odom.pose.pose.position.z
                print(f'  Frame {i:4d} | proc={processed:4d} '
                      f'skip={skipped:3d} | '
                      f'valid_px={valid_px:6d} | '
                      f'pts={len(combined.points):8d} | '
                      f'drone_z={z:.2f}m')

        except Exception as e:
            print(f'  Warning frame {i}: {e}')
            skipped += 1

    print(f'\nProcessed: {processed}  Skipped: {skipped}')
    print(f'Total points: {len(combined.points)}')

    if len(combined.points) == 0:
        print('ERROR: No points generated!')
        return

    # ── Post-processing ───────────────────────────────────────────────────────
    print(f'\nVoxel downsampling ({VOXEL_SIZE}m)...')
    combined = combined.voxel_down_sample(voxel_size=VOXEL_SIZE)
    print(f'After downsample: {len(combined.points)}')

    print('Removing outliers...')
    combined, _ = combined.remove_statistical_outlier(
        nb_neighbors=20, std_ratio=2.0)
    print(f'After outlier removal: {len(combined.points)}')

    print('Estimating normals...')
    combined.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.1, max_nn=30))
    combined.orient_normals_towards_camera_location(
        camera_location=np.array([0.0, 0.0, 50.0]))

    # ── Save ─────────────────────────────────────────────────────────────────
    print(f'\nSaving to {OUTPUT_PATH}...')
    o3d.io.write_point_cloud(OUTPUT_PATH, combined)
    print(f'Saved! ({len(combined.points)} points)')

    # ── Visualize ─────────────────────────────────────────────────────────────
    print('\nVisualizing... (press Q to quit)')
    o3d.visualization.draw_geometries(
        [combined],
        window_name='Mission Map',
        width=1280, height=720)

if __name__ == '__main__':
    main()
