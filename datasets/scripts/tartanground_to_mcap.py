"""Convert one TartanGround (omni) trajectory to a rosbag2 MCAP.

Run inside the devcontainer with ROS 2 sourced (uses rosbag2_py, no extra pip packages):
    python3 datasets/scripts/tartanground_to_mcap.py datasets/data/Hospital/Data_omni/P0005
    python3 datasets/scripts/tartanground_to_mcap.py <traj_dir> -o <bag_dir> [--no-images] [--max-frames N]

Output: datasets/bags/<env>.bag (a rosbag2 directory). One trajectory per env; use -o for others.

Topics written:
    /lidar                        sensor_msgs/PointCloud2       frame "lidar", 10 Hz
    /tf                           tf2_msgs/TFMessage            map -> base_link from ground-truth poses
    /tf_static                    tf2_msgs/TFMessage            base_link -> lidar, base_link -> camera_front
    /camera/image_raw/compressed  sensor_msgs/CompressedImage   PNG, lcam_front

Frame notes (verified by scan-overlap registration of consecutive scans plus a gravity check):
  * pose_lcam_front.txt holds TartanAir-convention poses [x y z qx qy qz qw]: NED world (z DOWN), and the body frame
    is x forward, y right, z down (FRD). Going up stairs makes z more negative.
  * The raw LiDAR .ply maps to that body frame as FRD = (lidar_y, lidar_x, -lidar_z), a proper rotation. Of all 48
    signed axis permutations this gives the best overlap of scans 3 frames apart (0.79) and puts the floor below the
    robot. (An earlier version read the poses as z-up and swapped lidar x/y instead: locally consistent, but the
    whole world came out vertically mirrored, e.g. stairs going up appeared to go down.)
  * Published in ROS conventions: map is ENU (z up) and base_link is FLU (x forward, y left, z up), so the cloud is
    (lidar_y, -lidar_x, lidar_z) and base_link -> lidar is identity. The LiDAR is assumed to sit at the pose origin.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Transform, TransformStamped
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

T0_NS = 1_700_000_000 * 10**9  # arbitrary non-zero epoch so TF lookups behave
DT_NS = 100_000_000  # 10 Hz

# offered_qos_profiles for a latched topic (history=keep_all, reliable, transient_local)
TF_STATIC_QOS = """- history: 3
  depth: 0
  reliability: 1
  durability: 1
  deadline:
    sec: 2147483647
    nsec: 4294967295
  lifespan:
    sec: 2147483647
    nsec: 4294967295
  liveliness: 1
  liveliness_lease_duration:
    sec: 2147483647
    nsec: 4294967295
  avoid_ros_namespace_conventions: false
"""


NED_TO_ENU = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)  # world: NED -> ENU
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])                                  # body: ROS FLU -> TartanAir FRD


def quat_to_matrix(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(m):
    """Rotation matrix -> [x y z w] (Shepperd's method)."""
    t = np.trace(m)
    if t > 0:
        s = 2 * np.sqrt(t + 1)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(1 + m[i, i] - m[j, j] - m[k, k])
        q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
        q[i] = s / 4
        q[j] = (m[j, i] + m[i, j]) / s
        q[k] = (m[k, i] + m[i, k]) / s
    return np.array(q)


def ned_pose_to_ros(p, q):
    """TartanAir pose (NED world, FRD body) -> map (ENU) -> base_link (FLU) translation and quaternion."""
    return NED_TO_ENU @ p, matrix_to_quat(NED_TO_ENU @ quat_to_matrix(q) @ FLU_TO_FRD)


def read_ply_xyz(path):
    """Binary little-endian PLY with float32 x y z only."""
    raw = path.read_bytes()
    marker = b"end_header\n"
    start = raw.index(marker) + len(marker)
    return np.frombuffer(raw, dtype="<f4", offset=start).reshape(-1, 3)


def stamp(t_ns):
    return Time(sec=t_ns // 10**9, nanosec=t_ns % 10**9)


def header(t_ns, frame):
    return Header(stamp=stamp(t_ns), frame_id=frame)


def make_tf(t_ns, parent, child, p, q):
    tf = TransformStamped(header=header(t_ns, parent), child_frame_id=child)
    tf.transform = Transform()
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, p)
    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
    return tf


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("traj_dir", type=Path)
    ap.add_argument("-o", "--output", type=Path, help="bag directory (default: datasets/bags/<env>.bag)")
    ap.add_argument("--no-images", action="store_true")
    ap.add_argument("--max-frames", type=int)
    args = ap.parse_args()

    traj = args.traj_dir.resolve()
    env = traj.parents[1].name
    out = args.output or Path(__file__).resolve().parents[1] / "bags" / f"{env}.bag"
    if out.exists():
        sys.exit(f"{out} already exists; remove it or pass -o")
    out.parent.mkdir(parents=True, exist_ok=True)

    scans = sorted((traj / "lidar").glob("*_lcam_front_lidar.ply"))
    poses = np.loadtxt(traj / "pose_lcam_front.txt")
    images = [] if args.no_images else sorted((traj / "image_lcam_front").glob("*.png"))
    n = min(len(scans), len(poses))
    if images:
        n = min(n, len(images))
    if args.max_frames:
        n = min(n, args.max_frames)
    if len({len(scans), len(poses)} | ({len(images)} if images else set())) > 1:
        print(f"warning: scans={len(scans)} poses={len(poses)} images={len(images)}; using {n}", file=sys.stderr)

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(out), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))

    def topic(name, type_, latched=False):
        # Humble takes a YAML string for offered_qos_profiles; Jazzy takes an id and a list of QoS objects.
        try:
            meta = rosbag2_py.TopicMetadata(
                name=name, type=type_, serialization_format="cdr",
                offered_qos_profiles=TF_STATIC_QOS if latched else "")
        except TypeError:
            from rosbag2_py._storage import QoS
            qos = [QoS(1).keep_all().reliable().transient_local()] if latched else []
            meta = rosbag2_py.TopicMetadata(
                id=0, name=name, type=type_, serialization_format="cdr", offered_qos_profiles=qos)
        writer.create_topic(meta)

    topic("/lidar", "sensor_msgs/msg/PointCloud2")
    topic("/tf", "tf2_msgs/msg/TFMessage")
    topic("/tf_static", "tf2_msgs/msg/TFMessage", latched=True)
    if images:
        topic("/camera/image_raw/compressed", "sensor_msgs/msg/CompressedImage")

    ident = ([0, 0, 0], [0, 0, 0, 1])
    static = [make_tf(T0_NS, "base_link", "lidar", *ident)]
    if images:
        static.append(make_tf(T0_NS, "base_link", "camera_front", *ident))
    writer.write("/tf_static", serialize_message(TFMessage(transforms=static)), T0_NS)

    fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate("xyz")]

    for i in range(n):
        t = T0_NS + i * DT_NS
        raw = read_ply_xyz(scans[i])
        xyz = np.column_stack((raw[:, 1], -raw[:, 0], raw[:, 2]))  # raw lidar -> base_link (FLU)
        xyz = np.ascontiguousarray(xyz, dtype="<f4")
        cloud = PointCloud2(header=header(t, "lidar"), height=1, width=len(xyz), fields=fields,
                            is_bigendian=False, point_step=12, row_step=12 * len(xyz),
                            data=xyz.tobytes(), is_dense=True)
        writer.write("/lidar", serialize_message(cloud), t)

        msg = TFMessage(transforms=[make_tf(t, "map", "base_link", *ned_pose_to_ros(poses[i, :3], poses[i, 3:]))])
        writer.write("/tf", serialize_message(msg), t)

        if images:
            img = CompressedImage(header=header(t, "camera_front"), format="png",
                                  data=images[i].read_bytes())
            writer.write("/camera/image_raw/compressed", serialize_message(img), t)

    del writer  # flush and close
    print(f"wrote {n} frames ({n / 10:.0f} s) to {out}")


if __name__ == "__main__":
    main()
