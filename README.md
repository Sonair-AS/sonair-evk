# sonair-evk

<p align="center">
    <img src="https://github.com/user-attachments/assets/90a13feb-4978-40c9-b9a4-e35ac17033ca">
</p>

The purpose of this public repository is mainly to serve as an example of how to
receive pointcloud data from the sonair-evk; and in particular it should serve
as an example to build upon if you wish to integrate the sonair-evk pointcloud
into an internal system. In order to visualize the pointcloud from a standalone
sonair-evk unit you can just use FoxGlove endpoint as described in the
sonair-evk instructions for use[....].

## The protocol
The sonair-evk unit will publish the pointcloud, along with a status flag as
[CoAP](https://en.wikipedia.org/wiki/Constrained_Application_Protocol) messages
over UDP. The IP address for these packages is configured by the user with
sonair configuration tool [....]. When used with a ROS publishing node we
currently have an arcitecture which looks like this:

```

                                                 +----------------------------------------------------------+
                                                 |                                                          |
   +--------------+                              |  +-------------+      +-------------------------------+  |
   |              |     +----------------+       |  |             |      |                               |  |
   |  sonair_evk  |-->--|  CoAP message  |---->--|--| CoAP Server |      | ROS Pointcloud publisher node |  |
   |              |     +----------------+       |  |             |      |                               |  |
   +--------------+                              |  +-------------+      +--------------------------------  |
                                                 |          |                                  |            |
                                                 |          +-->--[ Callback(pointcloud) ]-->--+            |
                                                 |                                                          |
                                                 |                                                          |
                                                 +----------------------------------------------------------+
```

Observe that the outer box on the right denotes a process bouundary, i.e. the
CoAP server and the ROS pointcloud publisher node run in the same process, the
ROS pointcloud publisher initializes a CoAP server with a callback which the
coap server will call for each received pointcloud - in order to pass the
pointclouds back to the ROS pointcloud publisher.


## The CoAP server
We have written our own little CoAP server which receives CoAP messages from the
sonair-evk unit, converts them to a pointcloud description and passes them back
to the ROS pointcloud publisher via a callback. The sole reason for doing it
like this is that the CoAP server should serve as an example/documentation of
how the messages sent from the sonair-evk unit can be converted to a pointcloud.
In a production setting it is probably better to use a mature CoAP server, like
e.g. [aiocoap](https://aiocoap.readthedocs.io/en/latest/). We have verified that
the setup also works with aiocoap.

The CoAP server is located in the
[ros/sonair_evk/sonar_evk/coap_server.py](https://github.com/Sonair-AS/sonair-evk/blob/main/ros/sonair_evk/sonair_evk/coap_server.py).
The functionality of the CoAP server coarsely consist of four steps:

1. We creat a socket on the correct port (that is currently hardcoded to 5683)
   and listen for messages.

2. We extract the CoAP specific header and assert that it agrees with what the
   sonair-evk unit sends.

3. When all the CoAP metadata has been processed the payload from the sonair-evk
   unit is passed to a free function which converts it to a pointcloud:
   [ros/sonair_evk/sonar_evk/convert_coap_payload_to_pointcloud.py](https://github.com/Sonair-AS/sonair-evk/blob/main/ros/sonair_evk/sonair_evk/convert_coap_payload_to_pointcloud.py).

4. The ROS pointcloud publisher callback is invoked with the pointcloud.



