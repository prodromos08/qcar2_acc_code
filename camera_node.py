#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


def clamp(x, a, b):
    return a if x < a else b if x > b else x


class LaneOffWhiteRightLock(Node):
    """
    White-mask lane detection using a bottom band.
    Policy: Right-edge lock
      - If right rail is present: lane center = right_x - (lane_width * right_bias_frac)
      - If only one rail is present: infer center using the estimated lane width

    Publishes:
      /lane_off   [-1..1]
      /lane_conf  [0..1]
      /lane_debug image
    """

    def __init__(self):
        super().__init__("lane_off_white_rightlock")

        self.declare_parameter("image_topic", "/camera/color_image")
        self.declare_parameter("lane_off_topic", "/lane_off")
        self.declare_parameter("lane_conf_topic", "/lane_conf")
        self.declare_parameter("debug_topic", "/lane_debug")

        self.declare_parameter("roi_top_frac", 0.45)
        self.declare_parameter("kill_far_frac", 0.30)

        # Fraction of lane width used when anchoring from the right edge.
        # 0.50 -> geometric center. Smaller -> shift center toward the right.
        self.declare_parameter("right_bias_frac", 0.50)

        self.declare_parameter("use_adaptive_L", True)
        self.declare_parameter("L_min", 200)
        self.declare_parameter("adaptive_L_offset", 25)
        self.declare_parameter("S_max", 90)

        self.declare_parameter("open_ksize", 3)
        self.declare_parameter("close_ksize", 7)

        self.declare_parameter("stitch_dots", True)
        self.declare_parameter("stitch_w", 31)
        self.declare_parameter("stitch_h", 5)

        self.declare_parameter("bottom_band_ratio", 0.18)
        self.declare_parameter("bottom_band_center_frac", 0.95)
        self.declare_parameter("guard_frac", 0.01)

        # Lane width estimation (used for right-lock and one-rail fallback).
        self.declare_parameter("lane_w_alpha", 0.25)
        self.declare_parameter("lane_w_fallback_frac", 0.75)
        self.declare_parameter("lane_w_min_frac", 0.55)
        self.declare_parameter("lane_w_max_frac", 0.95)

        # Output shaping.
        self.declare_parameter("invert_lr", False)
        self.declare_parameter("off_bias", 0.0)
        self.declare_parameter("off_gain", 1.0)

        self.declare_parameter("publish_debug", True)

        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.bridge = CvBridge()

        img_topic = self.get_parameter("image_topic").value
        off_topic = self.get_parameter("lane_off_topic").value
        conf_topic = self.get_parameter("lane_conf_topic").value
        dbg_topic = self.get_parameter("debug_topic").value

        self.sub = self.create_subscription(Image, img_topic, self.on_image, cam_qos)
        self.pub_off = self.create_publisher(Float32, off_topic, 10)
        self.pub_conf = self.create_publisher(Float32, conf_topic, 10)
        self.pub_dbg = self.create_publisher(Image, dbg_topic, 10)

        self.cx_filt = None
        self.cx_alpha = 0.20
        self.lane_w_px_est = None

        self.get_logger().info(
            f"LaneOffWhiteRightLock sub={img_topic} pub={off_topic},{conf_topic}"
        )

    def _decode(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            except Exception:
                return None

        if frame is None or frame.size == 0:
            return None

        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif frame.ndim == 3 and frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        return frame

    def _mask_white(self, roi):
        hls = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)
        L = hls[:, :, 1]
        S = hls[:, :, 2]

        S_max = int(self.get_parameter("S_max").value)

        if bool(self.get_parameter("use_adaptive_L").value):
            medL = int(np.median(L))
            off = int(self.get_parameter("adaptive_L_offset").value)
            L_min = int(clamp(medL + off, 120, 245))
        else:
            L_min = int(self.get_parameter("L_min").value)

        return ((L >= L_min) & (S <= S_max)).astype(np.uint8) * 255

    def on_image(self, msg):
        frame = self._decode(msg)
        if frame is None:
            return

        H, W = frame.shape[:2]

        y0 = int(clamp(float(self.get_parameter("roi_top_frac").value), 0.0, 0.95) * H)
        roi = frame[y0:H, :]

        mask = self._mask_white(roi)

        mh = mask.shape[0]
        mask[: int(float(self.get_parameter("kill_far_frac").value) * mh), :] = 0

        ok = int(self.get_parameter("open_ksize").value)
        ck = int(self.get_parameter("close_ksize").value)

        if ok >= 3:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ok, ok)),
            )
        if ck >= 3:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ck, ck)),
            )

        if bool(self.get_parameter("stitch_dots").value):
            sw = int(self.get_parameter("stitch_w").value)
            sh = int(self.get_parameter("stitch_h").value)
            sw = sw if sw % 2 == 1 else sw + 1
            sh = sh if sh % 2 == 1 else sh + 1
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                cv2.getStructuringElement(cv2.MORPH_RECT, (sw, sh)),
            )

        band_h = max(3, int(float(self.get_parameter("bottom_band_ratio").value) * mh))
        y_band = int(float(self.get_parameter("bottom_band_center_frac").value) * (mh - 1))
        y0b = max(0, y_band - band_h // 2)
        y1b = min(mh, y_band + band_h // 2)
        band = mask[y0b:y1b, :]

        col = (band > 0).sum(axis=0)

        if col.max() < 2:
            conf = 0.0
            cx = self.cx_filt if self.cx_filt is not None else (W / 2)
            mode = "lost"
            Lx = None
            Rx = None
        else:
            hits = col >= max(2, 0.3 * col.max())
            g = int(float(self.get_parameter("guard_frac").value) * W)
            hits[:g] = False
            hits[W - g:] = False
            xs = np.where(hits)[0]

            min_w = float(self.get_parameter("lane_w_min_frac").value) * W
            max_w = float(self.get_parameter("lane_w_max_frac").value) * W
            fallback_w = float(self.get_parameter("lane_w_fallback_frac").value) * W

            lane_w = self.lane_w_px_est if self.lane_w_px_est is not None else fallback_w
            lane_w = clamp(lane_w, min_w, max_w)

            if xs.size >= 2:
                Lx, Rx = int(xs[0]), int(xs[-1])

                w_meas = clamp(Rx - Lx, min_w, max_w)
                if self.lane_w_px_est is None:
                    self.lane_w_px_est = float(w_meas)
                else:
                    a = float(self.get_parameter("lane_w_alpha").value)
                    self.lane_w_px_est = (1 - a) * self.lane_w_px_est + a * float(w_meas)

                lane_w = clamp(self.lane_w_px_est, min_w, max_w)

                bias = float(self.get_parameter("right_bias_frac").value)
                cx = float(Rx) - bias * float(lane_w)

                conf = float(clamp(0.6 + 0.8 * ((Rx - Lx) / max(1.0, W)), 0.0, 1.0))
                mode = "right_lock"

            elif xs.size == 1:
                x = int(xs[0])
                if x < (W / 2):
                    cx = x + 0.5 * lane_w
                    mode = "one_left"
                else:
                    cx = x - 0.5 * lane_w
                    mode = "one_right"

                conf = 0.35
                Lx = x if x < (W / 2) else None
                Rx = x if x >= (W / 2) else None
            else:
                conf = 0.0
                cx = self.cx_filt if self.cx_filt is not None else (W / 2)
                mode = "hold"
                Lx = None
                Rx = None

        if self.cx_filt is None:
            self.cx_filt = float(cx)
        else:
            self.cx_filt = (1 - self.cx_alpha) * self.cx_filt + self.cx_alpha * float(cx)

        cx_use = float(self.cx_filt)
        if bool(self.get_parameter("invert_lr").value):
            cx_use = (W - 1) - cx_use

        off = (cx_use - (W / 2.0)) / (W / 2.0)
        off = float(self.get_parameter("off_gain").value) * (
            off + float(self.get_parameter("off_bias").value)
        )
        off = float(clamp(off, -1.0, 1.0))

        self.pub_off.publish(Float32(data=float(off)))
        self.pub_conf.publish(Float32(data=float(conf)))

        if bool(self.get_parameter("publish_debug").value):
            dbg = roi.copy()
            y_draw = int(clamp(y_band, 0, mh - 1))
            cv2.line(dbg, (0, y_draw), (W - 1, y_draw), (255, 0, 0), 1)

            if Lx is not None:
                cv2.circle(dbg, (int(Lx), y_draw), 6, (0, 255, 255), -1)
            if Rx is not None:
                cv2.circle(dbg, (int(Rx), y_draw), 6, (0, 255, 255), -1)

            cv2.circle(dbg, (int(cx_use), y_draw), 6, (0, 255, 0), -1)
            cv2.line(dbg, (W // 2, 0), (W // 2, mh - 1), (0, 0, 255), 1)
            cv2.putText(
                dbg,
                f"{mode} off={off:+.3f} conf={conf:.2f}",
                (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                1,
            )

            out = self.bridge.cv2_to_imgmsg(dbg, "bgr8")
            out.header = msg.header
            self.pub_dbg.publish(out)


def main():
    rclpy.init()
    n = LaneOffWhiteRightLock()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()