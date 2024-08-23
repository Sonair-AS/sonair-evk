import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import numpy as np
from .coap_server import CoapServer
from .sonair_evk_status import SonairEvkStatus
import os

class PointCloudNode(Node):
    def __init__(self):
        super().__init__("pointcloud_node")

        print("Processing node started.")

        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, "/adar/pointcloud", 1
        )

        self.status_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 1
        )

        self.coap_server_thread = CoapServer(self.publish_pointcloud, self.get_clock())

        self.host_name = os.getenv('HOST_NAME')
        if self.host_name is None:
            print("HOST_NAME environment variable is not set")
            self.host_name = "Sonair ADAR"
        
    def publish_pointcloud(self, points, status, timestamp):
        points = np.asarray(points)
        ros_dtype = PointField.FLOAT32

        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes()

        # The fields specify what the bytes represent. The first 4 bytes
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        fields = [
            PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate("xyz")
        ]
        fields.append(
            PointField(
                name="intensity", offset=3 * itemsize, datatype=ros_dtype, count=1
            )
        )        
        # The PointCloud2 message also has a header which specifies which
        # coordinate frame it is represented in.
        header = Header(frame_id="adar", stamp=timestamp.to_msg())

        pc = PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(
                itemsize * 4
            ),  # Every point consists of four float32s (x,y,z,intensity).
            row_step=(itemsize * 4 * points.shape[0]),
            data=data,
        )

        self.pointcloud_publisher.publish(pc)
        
        self.publish_status(status)


    def publish_status(self, status):
        diag_msg = self.to_diagnostic_msg(status)
        self.status_publisher.publish(diag_msg)

    def to_diagnostic_msg(self, status):
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()

        diag = DiagnosticStatus()
        diag.name = "ADAR Status"
        diag.hardware_id = self.host_name

        if status == SonairEvkStatus.Startup:
            diag.level = DiagnosticStatus.OK
            diag.message = "ADAR is starting up"
        elif status == SonairEvkStatus.Ok:
            diag.level = DiagnosticStatus.OK
            diag.message = "ADAR is running"
        elif status == SonairEvkStatus.ObjectInStopZone:
            diag.level = DiagnosticStatus.OK
            diag.message = "Object detected in stop zone"
        elif status == SonairEvkStatus.Error10 or status == SonairEvkStatus.Error11:
            diag.level = DiagnosticStatus.ERROR
            diag.message = f"ADAR error {status}"
        else:
            diag.level = DiagnosticStatus.WARN
            diag.message = "Unknown status"

        diag_msg.status.append(diag)

        return diag_msg


def main(args=None):
    rclpy.init(args=args)

    publisher = PointCloudNode()

    while True:
        try:
            rclpy.spin_once(publisher, timeout_sec=0.1)
        except SystemExit:
            pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print("Cleanup")
    publisher.destroy_node()
    del publisher
    rclpy.shutdown()


if __name__ == "__main__":
    main()


