# Built from the same arithmetic as the QRS diagram.
import math
import time


def main():
    clock_start = time.perf_counter()
    def now_ms():
        return int((time.perf_counter() - clock_start) * 1000)

    def stop():
        brick.motor("M3").brake()
        brick.motor("M4").brake()

    stop()
    x = 0.0
    y = 0.0
    S = 0.0
    theta = 0.0
    i = 0
    count = 9
    r = 0.028
    W = 0.154
    motor_gain = 0.1
    K_dist = 0.8
    K_angle = 8.0
    threshold = 0.015
    v_max = 0.12
    omega_max = 1.5
    max_power = 60.0
    goal_timeout = 30000
    run_timeout = 180000
    x_target = [0.4, 0.4, 0, 0, 0.4, 0.6, 1.0, 0.6, 0.6]
    y_target = [0, 0.4, 0.4, 0.8, 0.8, 0.8, 0.8, 0.4, 0]
    encL0 = brick.encoder("E3").read()
    encR0 = brick.encoder("E4").read()
    start_time = now_ms()
    failed = False
    with open("odometry_data_sim.txt", "w") as log:
        def log_sample():
            log.write("{:.6f},{:.6f},{}\n".format(100*x, 100*y, now_ms()-start_time))
            log.flush()
        try:
            log_sample()
            while i < count and now_ms()-start_time < run_timeout:
                goal_time = now_ms()
                dx = x_target[i] - x
                dy = y_target[i] - y
                dist = math.sqrt(dx * dx + dy * dy)
                course = math.atan2(dy, dx)
                err = course - theta
                err = err - 2 * math.pi * math.floor((err + math.pi) / (2 * math.pi))
                while dist > threshold and now_ms()-goal_time < goal_timeout and now_ms()-start_time < run_timeout:
                    v = K_dist * dist * math.cos(err)
                    omega = K_angle * err + K_dist * math.sin(err) * math.cos(err)
                    scale = max(1.0, max(abs(v) / v_max, abs(omega) / omega_max))
                    v = v / scale
                    omega = omega / scale
                    vL = v - omega * W / 2
                    vR = v + omega * W / 2
                    powerL = vL / (r * motor_gain)
                    powerR = vR / (r * motor_gain)
                    scale = max(1.0, max(abs(powerL), abs(powerR)) / max_power)
                    powerL = powerL / scale
                    powerR = powerR / scale
                    brick.motor("M3").setPower(int(powerL))
                    brick.motor("M4").setPower(int(powerR))
                    script.wait(20)
                    encL = (brick.encoder("E3").read() - encL0) * math.pi / 180
                    encR = (brick.encoder("E4").read() - encR0) * math.pi / 180
                    oldS = S
                    S = (encL + encR) * r / 2
                    dS = S - oldS
                    oldTheta = theta
                    theta = (encR - encL) * r / W
                    x = x + dS * math.cos((oldTheta + theta) / 2)
                    y = y + dS * math.sin((oldTheta + theta) / 2)
                    dx = x_target[i] - x
                    dy = y_target[i] - y
                    dist = math.sqrt(dx * dx + dy * dy)
                    course = math.atan2(dy, dx)
                    err = course - theta
                    err = err - 2 * math.pi * math.floor((err + math.pi) / (2 * math.pi))
                    log_sample()
                stop()
                if dist > threshold:
                    raise RuntimeError("Timed out at waypoint {}".format(i+1))
                i += 1
            if i < count:
                raise RuntimeError("Run timed out")
        finally:
            stop()
            script.wait(100)
            encL = (brick.encoder("E3").read() - encL0) * math.pi / 180
            encR = (brick.encoder("E4").read() - encR0) * math.pi / 180
            oldS = S
            S = (encL + encR) * r / 2
            dS = S - oldS
            oldTheta = theta
            theta = (encR - encL) * r / W
            x = x + dS * math.cos((oldTheta + theta) / 2)
            y = y + dS * math.sin((oldTheta + theta) / 2)
            log_sample()


if __name__ == "__main__":
    main()
