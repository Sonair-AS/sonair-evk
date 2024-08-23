The evaluation kit publishes a pointcloud and a status as an UDP message over
the CoAP protocol. The example coap_server.py illustrates one way to receive and
handle pointcloud published by the evaluation kit, this coap server
implementation is what is used in the sonair ROS node.

The coap server in coap_server.py is included in the sonair_evk example
respository in order to illustrate how the sonair-evk can integrated into
customers internal tools, for production use with a Python implementation it is
probably better to use a standard Python coap server implemetation like aiocoap.

The method CoapServer.convert_coap_msg_to_pointcloud() illustrates how the coap
message can be unpacked and the pointcloud along with a status flag are
extracted. The extracted pointcloud and the status are then passed to the
callback function which has been registered when the coap_server was
instantiated.



### Publishing with ros
The file point_cloud_publisher.py is an example ROS node which uses the example
coap server. The PointCloudNode publisher




### Configuring the IP address
The IP address to publish to should be configured in the sonair-evk
configuration tool, under the Point Cloud Publish Url setting.
