The evaluation kit publishes a pointcloud and a status as an UDP message over
the CoAP protocol. The example coap_server.py illustrates one way to receive and
handle pointcloud published by the evaluation kit, this coap server
implementation is what is used in the sonair ROS node.







### Configuring the IP address
The IP address to publish to should be configured in the sonair-evk
configuration tool, under the Point Cloud Publish Url setting.
