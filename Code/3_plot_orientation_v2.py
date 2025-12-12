import asyncio
import struct
import math
import time

from bleak import BleakScanner, BleakClient

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa

DEVICE_NAME = "Nano33BLE_IMU"
CHAR_UUID   = "12345678-1234-5678-1234-56789abcdef1"

latest_rpy = [0.0, 0.0, 0.0]


def rpy_to_rotmat(roll, pitch, yaw):
    # deg -> rad
    r = math.radians(roll)
    p = math.radians(pitch)
    y = math.radians(yaw)

    cy, sy = math.cos(y), math.sin(y)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)

    # ZYX
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
            roll, pitch, yaw = struct.unpack("<fff", data)
            latest_rpy[0] = roll
            latest_rpy[1] = pitch
            latest_rpy[2] = yaw

        await client.start_notify(CHAR_UUID, handle_notify)

        # ---- Matplotlib setup ----
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # define a small "board" shape in its local frame
        # a short box, longer in X, so front/back are different
        body_pts = [
            [ 0.6,  0.2,  0.1],  # front-top-right
            [ 0.6, -0.2,  0.1],  # front-top-left
            [-0.6, -0.2,  0.1],  # back-top-left
            [-0.6,  0.2,  0.1],  # back-top-right
            [ 0.6,  0.2, -0.1],  # front-bottom-right
            [ 0.6, -0.2, -0.1],  # front-bottom-left
            [-0.6, -0.2, -0.1],  # back-bottom-left
            [-0.6,  0.2, -0.1],  # back-bottom-right
        ]
        # edges to draw
        edges = [
            (0,1),(1,2),(2,3),(3,0),   # top
            (4,5),(5,6),(6,7),(7,4),   # bottom
            (0,4),(1,5),(2,6),(3,7)    # sides
        ]
        # and a nose line to show heading
        nose = [1.0, 0.0, 0.0]  # forward

        try:
            while True:
                ax.cla()
                ax.set_xlim([-1, 1])
                ax.set_ylim([-1, 1])
                ax.set_zlim([-1, 1])
                ax.set_xlabel("X")
                ax.set_ylabel("Y")
                ax.set_zlabel("Z")
                ax.set_title(f"R={latest_rpy[0]:.1f} P={latest_rpy[1]:.1f} Y={latest_rpy[2]:.1f}")

                # rotation
                R = rpy_to_rotmat(latest_rpy[0], latest_rpy[1], latest_rpy[2])

                # rotate body
                rot_pts = [apply_rot(R, p) for p in body_pts]
                for (i, j) in edges:
                    p1 = rot_pts[i]
                    p2 = rot_pts[j]
                    ax.plot([p1[0], p2[0]],
                            [p1[1], p2[1]],
                            [p1[2], p2[2]],
                            color="k", linewidth=1)

                # draw body axes too (optional)
                origin = [0, 0, 0]
                x_axis = apply_rot(R, [1,0,0])
                y_axis = apply_rot(R, [0,1,0])
                z_axis = apply_rot(R, [0,0,1])
                ax.plot([origin[0], x_axis[0]],
                        [origin[1], x_axis[1]],
                        [origin[2], x_axis[2]], color="r", linewidth=2)
                ax.plot([origin[0], y_axis[0]],
                        [origin[1], y_axis[1]],
                        [origin[2], y_axis[2]], color="g", linewidth=2)
                ax.plot([origin[0], z_axis[0]],
                        [origin[1], z_axis[1]],
                        [origin[2], z_axis[2]], color="b", linewidth=2)

                # draw nose to show heading clearly
                nose_world = apply_rot(R, nose)
                ax.plot([0, nose_world[0]],
                        [0, nose_world[1]],
                        [0, nose_world[2]],
                        color="m", linewidth=3)

                plt.pause(0.01)
                await asyncio.sleep(0.01)

        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            await client.stop_notify(CHAR_UUID)


if __name__ == "__main__":
    asyncio.run(main())
