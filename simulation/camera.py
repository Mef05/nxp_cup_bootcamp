"""
camera.py - Simulated Pixy2 camera for the NXP Cup race car.

Simulates what the Pixy2 Line Tracking module would see given the car's
pose and the world-space track boundary polylines.

Coordinate systems:
    World  : right-handed 2D plane.  X=east, Y=north.
             Z = 0 (ground), car camera is at height CAMERA_HEIGHT.
    Camera : X right, Y down (image convention), Z forward.
    Image  : pixel coords, origin top-left.
             Width=PIXY_W (0..78), Height=PIXY_H (0..50).
             Y=0 top (far from car), Y=50 bottom (close to car).

Projection:
    The camera is mounted at CAMERA_HEIGHT above the ground, tilted
    CAMERA_ANGLE_DEG below horizontal.  We use a simple pinhole
    perspective projection.  Visible ground distance range:
        near  ~ CAMERA_HEIGHT / tan(tilt + FOV_V/2)
        far   ~ CAMERA_HEIGHT / tan(tilt - FOV_V/2)  (clamped > 0)

Physical constants are ASSUMED - see config.py.
"""

from __future__ import annotations

import math
import random
from typing import List, Optional, Tuple

import numpy as np

from config import (
    CAMERA_ANGLE_DEG,
    CAMERA_FOV_H_DEG,
    CAMERA_FOV_V_DEG,
    CAMERA_HEIGHT,
    PIXY_H,
    PIXY_W,
)

# Pixel noise std dev (pixels) - simulates real sensor noise
_NOISE_STD = 0.8

# How many world points to sample along each boundary segment for projection
_SAMPLES_PER_SEGMENT = 60


class Pixy2Camera:
    """
    Simulated Pixy2 camera.

    Produces vectors in the same format as the real pixy_get_vectors() call:
        list of (x0, y0, x1, y1) tuples in Pixy2 pixel coordinates.

    Each tuple represents one detected line segment (boundary edge).
    x0,y0 = one end, x1,y1 = other end.  Pixy2 reports the bottom end
    (closer to car) as the point with higher Y.
    """

    def __init__(self) -> None:
        tilt_rad = math.radians(CAMERA_ANGLE_DEG)
        fov_v_rad = math.radians(CAMERA_FOV_V_DEG)
        fov_h_rad = math.radians(CAMERA_FOV_H_DEG)

        # Ground-distance range visible by the camera (along car forward axis)
        # near/far are forward distances on the ground plane from beneath the camera.
        self._near_dist = CAMERA_HEIGHT / math.tan(tilt_rad + fov_v_rad / 2.0)
        far_angle = tilt_rad - fov_v_rad / 2.0
        if far_angle > 0.0:
            self._far_dist = CAMERA_HEIGHT / math.tan(far_angle)
        else:
            self._far_dist = 20.0  # camera sees to horizon

        # Horizontal half-angle
        self._half_fov_h = fov_h_rad / 2.0
        self._tilt_rad = tilt_rad
        self._fov_v_rad = fov_v_rad

        # Focal length in pixels (computed from FOV and image size)
        self._fx = (PIXY_W / 2.0) / math.tan(fov_h_rad / 2.0)
        self._fy = (PIXY_H / 2.0) / math.tan(fov_v_rad / 2.0)

    # ------------------------------------------------------------------
    # Internal projection
    # ------------------------------------------------------------------

    def _world_to_pixel(
        self,
        car_x: float,
        car_y: float,
        car_theta: float,
        wx: float,
        wy: float,
    ) -> Optional[Tuple[float, float]]:
        """
        Project world point (wx, wy) onto the Pixy2 image plane given the
        car's pose.  Returns (px, py) in Pixy2 pixel coords, or None if the
        point is not visible.
        """
        # --- Transform to car frame (2D) ---
        dx = wx - car_x
        dy = wy - car_y
        # Rotate into car-local frame: forward=+X_cam, lateral=+Y_cam_right
        cf = math.cos(-car_theta)
        sf = math.sin(-car_theta)
        fwd = cf * dx - sf * dy   # forward distance (positive = in front of car)
        lat = sf * dx + cf * dy   # lateral distance (positive = right of car)

        # Reject points behind the camera or outside far distance
        if fwd < self._near_dist or fwd > self._far_dist:
            return None

        # --- Perspective projection ---
        # Camera is at height h above ground, tilted down by tilt_rad.
        # Ground point at (fwd, lat) in car frame, at height 0.
        # Camera ray direction in camera frame:
        #   Xc = lat  (right)
        #   Yc = CAMERA_HEIGHT (up from ground to camera, so ground is -Yc in cam)
        #   Zc = fwd  (forward)
        # After camera tilt (rotation around X axis by -tilt_rad):
        h = CAMERA_HEIGHT
        # Camera-frame coordinates before tilt compensation:
        #   cam_z_before = fwd, cam_y_before = h (height), cam_x_before = lat
        # After tilting camera down by tilt_rad around the camera X axis:
        cam_z = fwd * math.cos(self._tilt_rad) + h * math.sin(self._tilt_rad)
        cam_y = -fwd * math.sin(self._tilt_rad) + h * math.cos(self._tilt_rad)
        # Negate lat: the 2D rotation by -theta gives positive lat for points
        # to the LEFT of the car (e.g. car heading east -> left = +Y = +lat).
        # The camera image X axis points RIGHT, so cam_x must be negative for
        # left-of-car and positive for right-of-car. Without this negation the
        # boundaries are swapped in the image, inverting all steering decisions.
        cam_x = -lat

        if cam_z <= 0.0:
            return None

        # Project to normalised image coords
        nx = cam_x / cam_z
        ny = -cam_y / cam_z   # flip: image Y positive = downward

        # Convert to pixel coords
        px = nx * self._fx + PIXY_W / 2.0
        py = ny * self._fy + PIXY_H / 2.0

        # Clip to image bounds
        if px < 0 or px > PIXY_W - 1 or py < 0 or py > PIXY_H - 1:
            return None

        return px, py

    def _project_boundary(
        self,
        car_x: float,
        car_y: float,
        car_theta: float,
        boundary: List[Tuple[float, float]],
    ) -> Optional[Tuple[int, int, int, int]]:
        """
        Project a world boundary polyline into the image and fit a single
        representative line segment (as Pixy2 does with its CNN).

        Returns (x0, y0, x1, y1) in integer pixel coords, or None if the
        boundary is not visible.
        """
        pixels = []
        n = len(boundary)
        step = max(1, n // _SAMPLES_PER_SEGMENT)
        for i in range(0, n, step):
            pt = self._world_to_pixel(car_x, car_y, car_theta, boundary[i][0], boundary[i][1])
            if pt is not None:
                pixels.append(pt)

        if len(pixels) < 2:
            return None

        # Sort by Y (Pixy2: Y=0 top, Y=PIXY_H bottom)
        pixels.sort(key=lambda p: p[1])

        # Use the lowest and highest visible projected points as the vector
        # endpoints (mimicking Pixy2's line vector output).
        top_px, top_py = pixels[0]
        bot_px, bot_py = pixels[-1]

        # Add pixel-level noise
        noise = lambda: random.gauss(0, _NOISE_STD)
        top_px += noise(); top_py += noise()
        bot_px += noise(); bot_py += noise()

        # Clamp to image
        def clamp_px(v: float) -> int:
            return int(max(0, min(PIXY_W - 1, round(v))))

        def clamp_py(v: float) -> int:
            return int(max(0, min(PIXY_H - 1, round(v))))

        return clamp_px(bot_px), clamp_py(bot_py), clamp_px(top_px), clamp_py(top_py)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_vectors(
        self,
        car_x: float,
        car_y: float,
        car_theta: float,
        track_left: List[Tuple[float, float]],
        track_right: List[Tuple[float, float]],
    ) -> List[Tuple[int, int, int, int]]:
        """
        Simulate the Pixy2 get_vectors() call.

        Args:
            car_x, car_y  : world position of car (metres)
            car_theta     : car heading (radians)
            track_left    : left boundary polyline [(x, y), ...]
            track_right   : right boundary polyline [(x, y), ...]

        Returns:
            List of (x0, y0, x1, y1) tuples in Pixy2 pixel coordinates.
            Each tuple = one detected line vector.
            (x0, y0) = BOTTOM end (high Y, close to car)
            (x1, y1) = TOP end   (low Y, far from car)
            Returns [] if neither boundary is visible.
        """
        vectors = []

        left_vec = self._project_boundary(car_x, car_y, car_theta, track_left)
        right_vec = self._project_boundary(car_x, car_y, car_theta, track_right)

        if left_vec is not None:
            vectors.append(left_vec)
        if right_vec is not None:
            vectors.append(right_vec)

        return vectors
