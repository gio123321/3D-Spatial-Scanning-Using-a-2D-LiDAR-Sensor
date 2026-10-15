from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


def _robust_photo_stat_from_image(bgr: np.ndarray | None) -> tuple[float, np.ndarray] | None:
    if bgr is None:
        return None
    height, width = bgr.shape[:2]
    scale = min(1.0, 192.0 / max(height, width))
    if scale < 1.0:
        bgr = cv2.resize(
            bgr,
            (max(1, round(width * scale)), max(1, round(height * scale))),
            interpolation=cv2.INTER_AREA,
        )
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).reshape(-1, 3).astype(np.float32)
    luma = rgb @ np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)
    valid = (luma > 8) & (luma < 247) & np.all(rgb > 3, axis=1) & np.all(rgb < 252, axis=1)
    pixels = rgb[valid]
    if len(pixels) < 100:
        pixels = rgb
    channels = np.maximum(np.median(pixels, axis=0), 4.0)
    luminance = max(float(channels @ np.array([0.2126, 0.7152, 0.0722])), 4.0)
    chroma = channels / max(float(np.exp(np.mean(np.log(channels)))), 1e-6)
    return luminance, chroma


def _robust_photo_stat(path: str | Path) -> tuple[float, np.ndarray] | None:
    return _robust_photo_stat_from_image(cv2.imread(str(path), cv2.IMREAD_COLOR))


def estimate_photo_gains(
    paths: list[str | Path] | None = None,
    *,
    images: list[np.ndarray | None] | None = None,
    strength: float = 0.65,
    minimum: float = 0.72,
    maximum: float = 1.40,
) -> list[np.ndarray]:
    """Estimate bounded exposure and white-balance gains from thumbnail statistics.
    Supply either image paths or decoded BGR images.
    """

    count = len(images) if images is not None else len(paths or [])
    if images is not None:
        stats = [_robust_photo_stat_from_image(bgr) for bgr in images]
    else:
        stats = [_robust_photo_stat(path) for path in paths]
    valid = [value for value in stats if value is not None]
    if not valid:
        return [np.ones(3, dtype=np.float32) for _ in range(count)]
    target_luma = float(np.exp(np.median(np.log([value[0] for value in valid]))))
    target_chroma = np.exp(np.median(np.log(np.stack([value[1] for value in valid])), axis=0))
    amount = float(np.clip(strength, 0.0, 1.0))
    gains = []
    for stat in stats:
        if stat is None:
            gains.append(np.ones(3, dtype=np.float32))
            continue
        luminance, chroma = stat
        exposure = np.log(target_luma / luminance)
        white_balance = 0.30 * np.log(target_chroma / chroma)
        gain = np.exp(amount * (exposure + white_balance))
        gains.append(np.clip(gain, minimum, maximum).astype(np.float32))
    return gains


def apply_photo_gain(image_rgb: np.ndarray, gain: np.ndarray) -> np.ndarray:
    gain = np.asarray(gain, dtype=np.float32).reshape(1, 1, 3)
    return np.clip(image_rgb.astype(np.float32) * gain, 0, 255).astype(np.uint8)
