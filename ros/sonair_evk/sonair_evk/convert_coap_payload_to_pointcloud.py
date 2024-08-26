# Copyright (C), 2024, Sonair AS
#
# This software is free software, licensed according to the MIT License:https://en.wikipedia.org/wiki/MIT_License import struct
from typing import Tuple

from .sonair_evk_status import SonairEvkStatus

"""
This file contains the functions used to extract a pointcloud and status from
the payload in a CoAP message; it is assumed that the coap specific parts of
the message have been removed before calling these functions.

The payload is a bytes array which looks like this:

[ b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14, b15, b16, b17, b18, b19, .....]
  |____________|  |____________|  |______________|  |________________|  |________________|

   Status(uint)     xpos(float)      ypos(float)       zpos(float)       intensity(float)

                  |______________________________________________________________________| |_______

                                                point 0                                        point 1

All values in payload are encoded as little endian.
"""


def split_coap_payload(payload: bytes) -> Tuple[bytes, bytes]:
    """Split the payload in two byte arrays 

The first array has the four bytes encoding the status flag, 
and the second array has all the bytes for the pointcloud.

Returns:
    (bytes, bytes): Data for status, data for pointcloud
    """
    STATUS_OFFSET = 0
    STATUS_LENGTH = 4
    DATA_OFFSET = STATUS_OFFSET + STATUS_LENGTH

    status_data = payload[STATUS_OFFSET : STATUS_OFFSET + STATUS_LENGTH]
    point_data = payload[DATA_OFFSET:]

    return status_data, point_data


def convert_coap_payload_to_pointcloud(payload: bytes):
    """Extract the status and the pointcloud from the bytes in the payload. 

The return value is a tuple (points, status) where points is a list of points,
where each point is a four element list [x, y, z, intensity] and the status
is a SonairEvkStatus enum value.
     """
    state_data, point_data = split_coap_payload(payload)

    # Extract the state flag as a little endian 32 bit unsigned integer
    int_state, = struct.unpack("<I", state_data)
    try:
        state = SonairEvkStatus(int_state)
    except:
        print(f"Unexpected status integer: {int_state}")
        state = int_state

    points = []
    if len(point_data) > 0:
        assert (
            len(point_data) % 16 == 0
        ), f"Expected data length to be a multiple of 16, got {len(point_data)}"
        number_of_floats = len(point_data) // 4

        # Extract the four consecutive floating point values as little endian float32
        floats = struct.unpack(f"<{int(number_of_floats)}f", point_data)
        for i in range(0, len(floats), 4):
            x = floats[i]
            y = floats[i + 1]
            z = floats[i + 2]
            intensity = floats[i + 3]
            points.append([x, y, z, intensity])

    return points, state
