#!/usr/bin/env python3
"""
sound_localizer.py — TDOA-based sound source localization
Uses the Beast's stereo soundboard (hw:1,0) to estimate the horizontal
angle to a sound source at the moment of wake word detection.
"""

import numpy as np
import subprocess
import wave
import os
from datetime import datetime


# Physical constants
SPEED_OF_SOUND = 343.0          # m/s at ~20°C
MIC_SPACING    = 0.0381         # metres (1.5 inches)
SAMPLE_RATE    = 48000          # Hz — native rate of hw:1,0
MAX_DELAY_SAMPLES = int(np.ceil(MIC_SPACING / SPEED_OF_SOUND * SAMPLE_RATE))  # ~5

# Mics face the REAR of the robot, so we flip the angle
REAR_OFFSET_DEG = 180.0


def capture_stereo_snapshot(duration=1.0, device='hw:1,0'):
    """
    Record a short stereo clip from the soundboard.
    Returns (left_channel, right_channel) as numpy float arrays,
    or (None, None) on failure.
    """
    filename = f'/tmp/tdoa_{datetime.now().strftime("%Y%m%d_%H%M%S")}.wav'
    cmd = [
        'arecord',
        '-D', device,
        '-f', 'S16_LE',
        '-c', '2',
        '-r', str(SAMPLE_RATE),
        '-d', str(duration),
        filename
    ]
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       timeout=duration + 2.0)
        left, right = _read_stereo_wav(filename)
        return left, right
    except Exception as e:
        print(f'[sound_localizer] capture error: {e}')
        return None, None
    finally:
        if os.path.exists(filename):
            os.remove(filename)


def _read_stereo_wav(filename):
    """Split a stereo WAV into two float64 numpy arrays."""
    with wave.open(filename, 'rb') as wf:
        frames = wf.readframes(wf.getnframes())
    samples = np.frombuffer(frames, dtype=np.int16).astype(np.float64)
    left  = samples[0::2]
    right = samples[1::2]
    return left, right


def compute_tdoa(left, right):
    """
    Cross-correlation to find the sample delay between channels.
    Positive delay → sound arrived at left mic first → source is to the LEFT.
    Negative delay → sound arrived at right mic first → source is to the RIGHT.
    """
    # Normalise
    left  = left  - np.mean(left)
    right = right - np.mean(right)

    # GCC-PHAT (Generalised Cross-Correlation with Phase Transform)
    # More robust than plain cross-correlation for real-world audio
    n = len(left) + len(right) - 1
    fft_size = int(2 ** np.ceil(np.log2(n)))

    L = np.fft.rfft(left,  fft_size)
    R = np.fft.rfft(right, fft_size)

    cc = L * np.conj(R)
    denom = np.abs(cc)
    denom[denom < 1e-10] = 1e-10        # avoid divide-by-zero
    gcc_phat = np.fft.irfft(cc / denom, fft_size)

    # Only search within the physically possible delay range
    lags = np.concatenate([
        gcc_phat[-MAX_DELAY_SAMPLES:],
        gcc_phat[:MAX_DELAY_SAMPLES + 1]
    ])
    peak = np.argmax(lags) - MAX_DELAY_SAMPLES
    return int(peak)          # samples


def delay_to_angle(delay_samples):
    """
    Convert sample delay to angle in degrees.
    Returns angle in robot frame (0° = straight ahead).
    """
    delay_sec = delay_samples / SAMPLE_RATE
    # Clamp to physically valid range
    ratio = np.clip(delay_sec * SPEED_OF_SOUND / MIC_SPACING, -1.0, 1.0)
    angle_mic_frame = np.degrees(np.arcsin(ratio))
    # Mics face rear — rotate to robot frame
    angle_robot_frame = angle_mic_frame + REAR_OFFSET_DEG
    # Normalise to (-180, 180]
    if angle_robot_frame > 180.0:
        angle_robot_frame -= 360.0
    return angle_robot_frame


def localize():
    """
    Top-level call from the voice assistant.
    Returns estimated angle to sound source in degrees (robot frame),
    or None if localization failed.
    """
    left, right = capture_stereo_snapshot(duration=1.0)
    if left is None:
        return None

    delay = compute_tdoa(left, right)
    angle = delay_to_angle(delay)
    print(f'[sound_localizer] delay={delay} samples  angle={angle:.1f}°')
    return angle