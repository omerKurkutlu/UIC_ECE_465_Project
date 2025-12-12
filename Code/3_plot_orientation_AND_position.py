import asyncio
import struct
import math
from collections import deque

from bleak import BleakScanner, BleakClient

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa

DEVICE_NAME = "Nano33BLE_IMU"
CHAR_UUID   = "12345678-1234-5678-1234-56789abcdef1"

# [x, y, z, roll, pitch, yaw]
latest_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# keep a trailing path of positions
PATH_LEN = 500
path = deque(maxlen=PATH_LEN)


def rpy_to_rotmat(roll, pitch, yaw):
    # deg -> rad
    r = math.radians(roll)
    p = math.radians(pitch)
    y = math.radians(yaw)

    cy, sy = math.cos(y), math.sin(y)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)

    # ZYX rotation matrix, same convention as Arduino code
    R = [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ]
    return R


def apply_rot(R, v):
    return [
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    ]


async def main():
    print("Scanning for Nano33BLE_IMU...")
    dev = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name and DEVICE_NAME in d.name
    )
    if not dev:
        print("Device not found")
        return

    print(f"Found {dev.name} at {dev.address}")

    async with BleakClient(dev) as client:
        print("Connected. Subscribing...")

        def handle_notify(sender, data: bytearray):
            # 6 floats: x, y, z, roll, pitch, yaw
            x, y, z, roll, pitch, yaw = struct.unpack("<ffffff", data)
            latest_state[0] = x
            latest_state[1] = y
            latest_state[2] = z
            latest_state[3] = roll
            latest_state[4] = pitch
            latest_state[5] = yaw
            path.append((x, y, z))

        await client.start_notify(CHAR_UUID, handle_notify)

        # ---- Matplotlib setup ----
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # define a small "board" in its local frame
        body_pts = [
            [ 0.6,  0.2,  0.1],
            [ 0.6, -0.2,  0.1],
            [-0.6, -0.2,  0.1],
            [-0.6,  0.2,  0.1],
            [ 0.6,  0.2, -0.1],
            [ 0.6, -0.2, -0.1],
            [-0.6, -0.2, -0.1],
            [-0.6,  0.2, -0.1],
        ]
        edges = [
            (0,1),(1,2),(2,3),(3,0),   # top
            (4,5),(5,6),(6,7),(7,4),   # bottom
            (0,4),(1,5),(2,6),(3,7)    # sides
        ]

        nose = [1.0, 0.0, 0.0]  # forward

        try:
            while True:
                ax.cla()

                x, y, z, roll, pitch, yaw = latest_state

                # ---- compute dynamic limits from path + current position ----
                if path:
                    xs = [p[0] for p in path] + [x]
                    ys = [p[1] for p in path] + [y]
                    zs = [p[2] for p in path] + [z]

                    min_x, max_x = min(xs), max(xs)
                    min_y, max_y = min(ys), max(ys)
                    min_z, max_z = min(zs), max(zs)

                    # add a margin
                    margin = 0.5
                    min_x -= margin; max_x += margin
                    min_y -= margin; max_y += margin
                    min_z -= margin; max_z += margin

                    # optionally keep cube-ish aspect
                    span = max(max_x - min_x, max_y - min_y, max_z - min_z)
                    cx = 0.5 * (min_x + max_x)
                    cy = 0.5 * (min_y + max_y)
                    cz = 0.5 * (min_z + max_z)

                    half = span / 2.0
                    ax.set_xlim([cx - half, cx + half])
                    ax.set_ylim([cy - half, cy + half])
                    ax.set_zlim([cz - half, cz + half])
                else:
                    # fallback when we have almost no data yet
                    ax.set_xlim([-2, 2])
                    ax.set_ylim([-2, 2])
                    ax.set_zlim([-2, 2])

                ax.set_xlabel("X (m)")
                ax.set_ylabel("Y (m)")
                ax.set_zlabel("Z (m)")
                ax.set_title(
                    f"Pos=({x:.2f}, {y:.2f}, {z:.2f})  "
                    f"R={roll:.1f} P={pitch:.1f} Y={yaw:.1f}"
                )

                R = rpy_to_rotmat(roll, pitch, yaw)

                # ----- draw body at (x, y, z) -----
                rot_pts = []
                for p in body_pts:
                    pr = apply_rot(R, p)
                    rot_pts.append([pr[0] + x, pr[1] + y, pr[2] + z])

                for (i, j) in edges:
                    p1 = rot_pts[i]
                    p2 = rot_pts[j]
                    ax.plot(
                        [p1[0], p2[0]],
                        [p1[1], p2[1]],
                        [p1[2], p2[2]],
                        linewidth=1,
                    )

                origin = [x, y, z]
                x_axis = apply_rot(R, [0.5, 0, 0])
                y_axis = apply_rot(R, [0, 0.5, 0])
                z_axis = apply_rot(R, [0, 0, 0.5])

                ax.plot(
                    [origin[0], origin[0] + x_axis[0]],
                    [origin[1], origin[1] + x_axis[1]],
                    [origin[2], origin[2] + x_axis[2]],
                    linewidth=2,
                )
                ax.plot(
                    [origin[0], origin[0] + y_axis[0]],
                    [origin[1], origin[1] + y_axis[1]],
                    [origin[2], origin[2] + y_axis[2]],
                    linewidth=2,
                )
                ax.plot(
                    [origin[0], origin[0] + z_axis[0]],
                    [origin[1], origin[1] + z_axis[1]],
                    [origin[2], origin[2] + z_axis[2]],
                    linewidth=2,
                )

                nose_world = apply_rot(R, nose)
                ax.plot(
                    [origin[0], origin[0] + nose_world[0]],
                    [origin[1], origin[1] + nose_world[1]],
                    [origin[2], origin[2] + nose_world[2]],
                    linewidth=3,
                )

                # draw trajectory
                if len(path) > 1:
                    xs = [p[0] for p in path]
                    ys = [p[1] for p in path]
                    zs = [p[2] for p in path]
                    ax.plot(xs, ys, zs, linewidth=1)

                plt.pause(0.01)
                await asyncio.sleep(0.01)

        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            await client.stop_notify(CHAR_UUID)


if __name__ == "__main__":
    asyncio.run(main())
