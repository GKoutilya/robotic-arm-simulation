"""
Conversions between numpy arrays / src.camera.camera_sim.CameraFrame and their
ROS2 message equivalents (sensor_msgs/Image, arm_interfaces/CameraFrame).
"""
import numpy as np
from sensor_msgs.msg import Image

from arm_interfaces.msg import CameraFrame as CameraFrameMsg
from src.camera.camera_sim import CameraFrame


def numpy_to_image_msg(array, encoding, header=None):
    msg = Image()
    if header is not None:
        msg.header = header
    msg.height = array.shape[0]
    msg.width = array.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    channels = array.shape[2] if array.ndim == 3 else 1
    msg.step = array.shape[1] * array.itemsize * channels
    msg.data = array.tobytes()
    return msg


def image_msg_to_numpy(msg, dtype, channels=1):
    arr = np.frombuffer(msg.data, dtype=dtype)
    if channels > 1:
        arr = arr.reshape((msg.height, msg.width, channels))
    else:
        arr = arr.reshape((msg.height, msg.width))
    return arr


def camera_frame_to_msg(frame: CameraFrame, header) -> CameraFrameMsg:
    """Pack a captured CameraFrame (rgb/depth_buffer/segmentation only --
    view/projection matrices are NOT transmitted, since the camera is a fixed
    known simulated one; perception_node recomputes them from src.config.CAMERA)."""
    msg = CameraFrameMsg()
    msg.header = header
    msg.rgb = numpy_to_image_msg(frame.rgb.astype(np.uint8), 'rgb8', header)
    msg.depth_buffer = numpy_to_image_msg(frame.depth_buffer.astype(np.float32), '32FC1', header)
    msg.segmentation = numpy_to_image_msg(frame.segmentation.astype(np.int32), '32SC1', header)
    return msg


def msg_to_camera_frame(msg: CameraFrameMsg, view_matrix, projection_matrix) -> CameraFrame:
    """Reconstruct a CameraFrame from a received message plus locally-known
    camera matrices, so the existing object_detector functions work unchanged."""
    rgb = image_msg_to_numpy(msg.rgb, np.uint8, channels=3)
    depth_buffer = image_msg_to_numpy(msg.depth_buffer, np.float32, channels=1).astype(np.float64)
    segmentation = image_msg_to_numpy(msg.segmentation, np.int32, channels=1).astype(np.int64)
    height, width = segmentation.shape
    return CameraFrame(
        rgb=rgb,
        depth=None,  # unused downstream: find_objects_by_segmentation/pixel_to_world_coords only need depth_buffer
        depth_buffer=depth_buffer,
        segmentation=segmentation,
        view_matrix=view_matrix,
        projection_matrix=projection_matrix,
        width=width,
        height=height,
    )
