#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import csv
import time
from mavros_msgs.msg import OverrideRCIn
# set limit
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

# map to pwm
def map_to_pwm_x(n, pwm_min_x=1440, pwm_mid_x=1500, pwm_max_x=1560, deadband=0.023):   # need to find a better parameter and rang
    n = clamp(n, -1.0, 1.0)
    if abs(n) < deadband:
        return pwm_mid_x
    if n >= 0:
        pwm = pwm_mid_x + n * (pwm_max_x - pwm_mid_x)
    else:
        pwm = pwm_mid_x + n * (pwm_mid_x - pwm_min_x)
    return int(clamp(int(round(pwm)), pwm_min_x, pwm_max_x))

def map_to_pwm_y(n, pwm_min_y=1464, pwm_mid_y=1500, pwm_max_y=1536, deadband=0.023):   # need to find a better parameter and rang
    n = clamp(n, -1.0, 1.0)
    if abs(n) < deadband:
        return pwm_mid_y
    if n >= 0:
        pwm = pwm_mid_y + n * (pwm_max_y - pwm_mid_y)
    else:
        pwm = pwm_mid_y + n * (pwm_mid_y - pwm_min_y)
    return int(clamp(int(round(pwm)), pwm_min_y, pwm_max_y))

def map_to_pwm_z(n, pwm_min_z=1464, pwm_mid_z=1500, pwm_max_z=1536, deadband=0.023):   # need to find a better parameter and rang
    n = clamp(n, -1.0, 1.0)
    if abs(n) < deadband:
        return pwm_mid_z
    if n >= 0:
        pwm = pwm_mid_z + n * (pwm_max_z - pwm_mid_z)
    else:
        pwm = pwm_mid_z + n * (pwm_mid_z - pwm_min_z)
    return int(clamp(int(round(pwm)), pwm_min_z, pwm_max_z))


def map_to_pwm_yaw(n, pwm_min_yaw=1464, pwm_mid_yaw=1500, pwm_max_yaw=1536, deadband=0.023):   # need to find a better parameter and rang
    n = clamp(n, -1.0, 1.0)
    if abs(n) < deadband:
        return pwm_mid_yaw
    if n >= 0:
        pwm = pwm_mid_yaw + n * (pwm_max_yaw - pwm_mid_yaw)
    else:
        pwm = pwm_mid_yaw + n * (pwm_mid_yaw - pwm_min_yaw)
    return int(clamp(int(round(pwm)), pwm_min_yaw, pwm_max_yaw))



# define publish function
def publish_pwm_dict(pub, ch_pwm):
    msg = OverrideRCIn()
    msg.channels = [0] * 18
    for ch, pwm in ch_pwm.items():
        idx = int(ch) - 1
        if 0 <= idx < 18:
            msg.channels[idx] = int(pwm)
    pub.publish(msg)
# define node
def main():
    rospy.init_node("velfile_to_rc_override")

    # path to csv file
    csv_path = "/home/zeb/test-8/eight-thurster/src/real_test/data/all_vlocity_180s.csv"

    # parameters
    pwm_min, pwm_mid, pwm_max = 1100, 1500, 1900
    deadband = 0.023
    vmax_x = 0.424593  # m/s need to test
    vmax_y = 0.147  # m/s need to test
    vmax_z = 0.2  # m/s need to test
    yaw_max = 0.334  # rad/s need to test
    ch_forward, ch_lateral, ch_vertical, ch_yaw = 5, 6, 3, 4    # channel define need test
    realtime_play = True
    play_rate_hz = 20.0

    pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
    

    # 读取 CSV
    rows = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                rows.append({
                    "t": float(r["time"]),
                    "vx": float(r["vx"]),
                    "vy": float(r["vy"]),
                    "vz": float(r["vz"]),
                    "yaw": float(r["yaw"]),
                })
            except ValueError:
                continue

    if not rows:
        rospy.logerr("CSV 为空或无法解析有效行。")
        return

    rows.sort(key=lambda x: x["t"])
    t0 = rows[0]["t"]

    rospy.loginfo("[velfile_to_rc_override] rows: %d, t0=%.3f, realtime=%s",
                  len(rows), t0, str(realtime_play))

    start_wall = time.time()
    rate = rospy.Rate(play_rate_hz) if not realtime_play else None

    for r in rows:
        if rospy.is_shutdown():
            break

        # 实时播放
        if realtime_play:
            target = (r["t"] - t0)
            while not rospy.is_shutdown():
                elapsed = time.time() - start_wall
                if elapsed >= target - 1e-3:
                    break
                time.sleep(0.001)

        # 归一化
        nx = clamp(r["vx"] / vmax_x, -1.0, 1.0)
        ny = clamp(r["vy"] / vmax_y, -1.0, 1.0)
        nz = clamp(r["vz"] / vmax_z , -1.0, 1.0)
        nr = clamp(r["yaw"] / yaw_max, -1.0, 1.0)

        # 映射为 PWM
        pwm_forward = map_to_pwm_x(nx)
        pwm_lateral = map_to_pwm_y(ny)
        pwm_vertical = map_to_pwm_z(nz)
        pwm_yaw = map_to_pwm_yaw(nr)

        publish_pwm_dict(pub, {
            ch_forward: pwm_forward,
            ch_lateral: pwm_lateral,
            ch_vertical: pwm_vertical,
            ch_yaw: pwm_yaw
        })

        if not realtime_play:
            rate.sleep()

    rospy.loginfo("[velfile_to_rc_override] finished playback. Sending neutral once.")
    publish_pwm_dict(pub, {
        ch_forward: pwm_mid,
        ch_lateral: pwm_mid,
        ch_vertical: pwm_mid,
        ch_yaw: pwm_mid
    })

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
