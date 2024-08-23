from typing import Tuple, List

import struct
from threading import Thread
import socket

from sonair_evk.sonair_evk_status import SonairEvkStatus
from .convert_coap_payload_to_pointcloud import convert_coap_payload_to_pointcloud

class CoapServer:
    MESSAGE_TYPE = 0x01  # Non-confirmable message
    REQUEST_POST = 0x02  # POST
    EXPECTED_PATH = "point_cloud"
    COAP_VERSION = 1
    COAP_HEADER_LENGTH = 4


    def __init__(self, coap_subscriber_callback, clock):
        self.coap_subscriber_callback = coap_subscriber_callback
        self.clock = clock

        self.server_thread = Thread(target=self.run, name="coap_server", daemon=True)
        print("CoAP server starting.")
        print(f"Clock now: {self.clock.now()}")
        self.server_thread.start()
        self.last_msg_id = None

    def run(self):
        print("CoAP server started.")
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            coap_port = 5683  # https://www.netify.ai/resources/protocols/coap
            server.bind(("0.0.0.0", coap_port))
            print(f"Waiting for messages on {coap_port}...")
            cnt = 0
            while True:
                try:
                    # Receive a CoAP message.
                    msg, addr = server.recvfrom(2048)
                    timestamp = self.clock.now()
                    cnt += 1
                    # Print approx once per minute (expect 10 msgs per second)
                    if cnt % 600 == 0:
                        print(f"Received {cnt} messages")

                    # Convert the CoAP message to a PointCloud2 message.
                    points, status = self.convert_coap_msg_to_pointcloud(msg)

                    self.coap_subscriber_callback(points, status, timestamp)
                except Exception as e:
                    print(f"Error: {e} occurred while processing message.")
        except Exception as e:
            print(f"CoAP server stopped {e}")

    @staticmethod
    def unpack_coap_protocol(byte) -> Tuple[int, int, int]:
        """Unpack the first byte in the coap header

        In the assert_coap_header() function we assert that coap_protocol_version == 1, 
        msg_type == 1 (Non-confirmable)

        Returns:
            coap_protocol_version, coap_message_type, token_length
        """
        version = (byte & 0xC0) >> 6
        msg_type = (byte & 0x30) >> 4
        token_length = byte & 0x0F
        return version, msg_type, token_length

    @staticmethod
    def unpack_coap_request(byte) -> Tuple[int, int]:
        """Will unpack the second byte in the coap header

        The assert_coap_header() function asserts that code == 2 (POST REQUET), 
        the class value is ignored.

        Returns:
            class(int), code(int): class, code
        """
        cls = (byte >> 5) & 0x07
        code = byte & 0x1F
        return cls, code

    @staticmethod
    def assert_coap_header(msg) -> int:
        """Assert that the coap header is as expected

        For further details of the coap header see: 
        https://en.wikipedia.org/wiki/Constrained_Application_Protocol

        Returns:
            message_id(int): The message ID as an integer
        """
        version, msg_type, token_length = CoapServer.unpack_coap_protocol(msg[0])
        _, request_type = CoapServer.unpack_coap_request(msg[1])
        (msg_id,) = struct.unpack("<H", msg[2:4])

        assert (
            version == CoapServer.COAP_VERSION
            ), f"Only CoAP version {CoapServer.COAP_VERSION} is supported, got {version}"
        assert (
            msg_type == CoapServer.MESSAGE_TYPE
        ), f"Only CoAP v1 non-confirmable messages are supported, got {msg_type}"
        assert (
            token_length == 0
        ), f"Expected token length to be 0, got {token_length}"

        assert (
            request_type == CoapServer.REQUEST_POST
        ), f"Only POST requests are supported, got {request_type}"
        return msg_id



    @staticmethod
    def assert_path(msg) -> int:
        """Will assert that the path part of the message is corect and return start of payload

        The path published by sonair-evk is '/point_cloud' - here we skip the leading '/', and 
        then assert that we have the string 'point_cloud', before the actual payload start there is 0xff byte.

        Returns:
            offset(int): The offset into the msg buffer where the actual payload starts.
        """
        # The +1 here is to skip the leading '/' in the path
        PATH_OFFSET = CoapServer.COAP_HEADER_LENGTH + 1
        path_length = msg[PATH_OFFSET:].index(0xFF)
        actual_path = msg[PATH_OFFSET: PATH_OFFSET + path_length].decode("ascii")

        assert (
            actual_path == CoapServer.EXPECTED_PATH
        ), f"Expected path '{CoapServer.EXPECTED_PATH}' but got '{actual_path}'"
        return PATH_OFFSET + path_length + 1


    #  See  https://en.wikipedia.org/wiki/Constrained_Application_Protocol
    def extract_coap_payload(self, msg) -> bytes:
        """Extract the payload from the coap message

        Will split out the header/path and payload parts of the message. We assert that
        the header/path is in accordance to what the soanir-evk posts, then the actual 
        payload is extracted and returned as a bytes array. This bytes array should then
        be passed to the free function convert_coap_payload_to_pointcloud() which parse
        the bytes and create a status value and a pointcloud.

        Returns:
            payload(bytes): The payload as bytes()
        """
        msg_id = CoapServer.assert_coap_header(msg[:CoapServer.COAP_HEADER_LENGTH])
        if self.last_msg_id is not None and msg_id != self.last_msg_id + 1:
            print(f"Missed message id {self.last_msg_id} -> {msg_id}")

        payload_offset = CoapServer.assert_path(msg)
        return msg[payload_offset:]


    def convert_coap_msg_to_pointcloud(self, msg) -> Tuple[SonairEvkStatus, List[List[float]]]:
        """Convert the coap message to a pointcloud and a status flag

        The convert_coap_msg_to_pointcloud will split out all header/path information 
        from the coap message and then pass the payload to the free function
        convert_coap_payload_to_pointcloud(). 
     
        Returns:
            (SonairEvkStatus, List[List[float]]): Status and pointcloud        
        """
        payload = self.extract_coap_payload(msg)
        return convert_coap_payload_to_pointcloud(payload)


