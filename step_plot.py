#!/usr/bin/env python3
import threading
from collections import deque
from typing import Dict, Deque, Tuple, List

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ---------- konfiguracja ---------------------------------------------------
J_NAMES: List[str] = [
    "front_left_first_joint",
    "front_left_second_joint",
    "front_left_third_joint",
]
BUFFER_SECONDS   = 10.0
PLOT_REFRESH_HZ  = 1.0
MAX_SAMPLES      = int(BUFFER_SECONDS * 200)
# ---------------------------------------------------------------------------


class JointPlotter(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_plotter")
        self.pos_buf: Dict[str, Deque[Tuple[float, float]]] = {
            j: deque(maxlen=MAX_SAMPLES) for j in J_NAMES}
        self.eff_buf: Dict[str, Deque[Tuple[float, float]]] = {
            j: deque(maxlen=MAX_SAMPLES) for j in J_NAMES}

        self.create_subscription(JointState, "/joint_states",
                                 self.joint_cb, 50)

        # matplotlib setup (fig & animation, BUT show() stays in main thread)
        plt.style.use("ggplot")
        self.fig = plt.figure(figsize=(10, 6))
        self.ani = animation.FuncAnimation(
            self.fig, self.animate,
            interval=1000.0 / PLOT_REFRESH_HZ
        )

    # ----------------- callbacks -------------------------------------------
    def joint_cb(self, msg: JointState) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for name, pos, eff in zip(msg.name, msg.position, msg.effort):
            if name in J_NAMES:
                self.pos_buf[name].append((stamp, pos))
                self.eff_buf[name].append((stamp, eff))
                
        

        
    # -----------------------------------------------------------------------
    def animate(self, _frame) -> None:
        ax1 = plt.gca()
        ax1.cla()
        ax2 = ax1.twinx()

        now = self.get_clock().now().seconds_nanoseconds()[0]
        t_min = now - BUFFER_SECONDS

        eff_all, pos_all = [], []
        for j in J_NAMES:
            e_filtered = [(t, e) for t, e in self.eff_buf[j] if t >= t_min]
            p_filtered = [(t, p) for t, p in self.pos_buf[j] if t >= t_min]
            if not e_filtered or not p_filtered:
                continue

            t_e, eff = zip(*e_filtered)
            t_p, pos = zip(*p_filtered)

            ax1.plot(t_e, eff, label=f"{j} torque")
            ax2.plot(t_p, pos, '--', label=f"{j} pos")

            eff_all.extend(eff)
            pos_all.extend(pos)

        if not eff_all or not pos_all:
            return

        global_min = min(min(eff_all), min(pos_all))
        global_max = max(max(eff_all), max(pos_all))
        ax1.set_ylim(global_min, global_max)
        ax2.set_ylim(global_min, global_max)
        ax1.set_xlim(t_min, now)

        ax1.set_xlabel("czas [s]")
        ax1.set_ylabel("moment [Nm]")
        ax2.set_ylabel("pozycja [rad]")
        ax1.set_title("Moment (linia ciągła) vs Pozycja (linia przerywana)")
        ax1.legend(loc="upper left")
        ax1.grid(True)


# -------------------------- main ------------------------------------------
def ros_spin(node: Node) -> None:
    """Oddzielny wątek z rclpy.spin()."""
    rclpy.spin(node)
    node.destroy_node()

def main() -> None:
    rclpy.init()
    node = JointPlotter()

    # Start ROS-owy executor w tle,
    # GUI matplotlib pozostaje w wątku głównym (bez problemu z Tk)
    threading.Thread(target=ros_spin, args=(node,), daemon=True).start()

    try:
        plt.show()          # ← blokujące wywołanie w MAIN THREAD
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
