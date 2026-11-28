"""Waffle action/state schema: base twist, four arm angles, gripper position."""
import numpy as np

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint']
NAMES = ['linear_m_s', 'yaw_rad_s', *JOINTS]
LOW = np.array([-.12, -.65, -.9*np.pi, -.57*np.pi, -.3*np.pi, -.57*np.pi, -.010], np.float32)
HIGH = np.array([.12, .65, .9*np.pi, .5*np.pi, .44*np.pi, .65*np.pi, .019], np.float32)
SCHEMA = 'waffle-twist-joint-position-v1'


def normalized(values):
    return np.clip(2*(np.asarray(values)-LOW)/(HIGH-LOW)-1, -1, 1)


def physical(values):
    return LOW + (np.clip(values, -1, 1)+1)*(HIGH-LOW)/2
