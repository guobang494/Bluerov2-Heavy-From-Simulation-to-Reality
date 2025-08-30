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
def map_to_pwm(n, pwm_min=1440, pwm_mid=1500, pwm_max=1560, deadband=0.023):   # need to find a better parameter and rang
    n = clamp(n, -1.0, 1.0)
    if abs(n) < deadband:
        return pwm_mid
    if n >= 0:
        pwm = pwm_mid + n * (pwm_max - pwm_mid)
    else:
        pwm = pwm_mid + n * (pwm_mid - pwm_min)
    return int(clamp(int(round(pwm)), pwm_min, pwm_max))

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
    vmax_x = 1.09  # m/s need to test
    vmax_y = 0.141  # m/s need to test
    vmax_z = 0.03  # m/s need to test
    yaw_max = 0.04  # rad/s need to test
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
        pwm_forward = map_to_pwm(nx, pwm_min, pwm_mid, pwm_max, deadband)
        pwm_lateral = map_to_pwm(ny, pwm_min, pwm_mid, pwm_max, deadband)
        pwm_vertical = map_to_pwm(nz, pwm_min, pwm_mid, pwm_max, deadband)
        pwm_yaw = map_to_pwm(nr, pwm_min, pwm_mid, pwm_max, deadband)

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
