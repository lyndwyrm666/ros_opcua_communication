# ros_opcua_impl_freeopcua

This package provides OPC UA client implementation for ROS.
The client can connect to an OPC UA server and access simple data from it.

## Getting started with ros_opcua_impl_freeopcua

To get started with OPC UA client implementation from this repository first an OPC UA server should be started.
For testing one can use example server from [freeopcua](https://github.com/FreeOpcUa/freeopcua)  library.
You can start it using following commands:

```
roscd && cd ..
./build/ros_opcua_impl_freeopcua/freeopcua/bin/example_server
```
Then start your client using:
```
ros2 launch ros_opcua_impl_freeopcua client.launch.py
```

**NOTE**: Here is server run on the same computer as the server.
If they are running on diffrent computers, instead of `localhost` insert DNS name or IP address of the computer where server is running.
It is also recommended to cross check all values and server using *UaExpert OPC UA client*.

### About examples
If you provide `nodeId` in service calls the `qualifiedName` is not required and it can be left out.

### Example: Reading and Writing Variables
Use following services to connect to a server, read and write the data:
```
ros2 service call /opcua/connect ros_opcua_srvs/srv/Connect "{endpoint: 'opc.tcp://IP-Address:port'}"
ros2 service call /opcua/opcua_client/Read "{node:
  nodeId: 'ns=2;i=2001'
  qualifiedName: 'MyVariable'}"
ros2 service call /opcua/opcua_client/Write "{node:
  nodeId: 'ns=2;i=2001'
  qualifiedName: 'MyVariable'
data: {type: 'uint32', bool_d: false, int8_d: 0, uint8_d: 0, int16_d: 0, uint16_d: 0, int32_d: 0,
  uint32_d: 100, int64_d: 0, uint64_d: 0, float_d: 0.0, double_d: 0.0, string_d: ''}}"
```
After this lines you should see value changes in OPC UA server of the variable `Objects->NewObject->MyVariable` to `100`.
(ATTENTION: you should be fast to check the value, because the example server changes it every few seconds!)

### Example: Subscribe to a Variable
```
ros2 service call /opcua/opcua_client/Connect "{endpoint: 'opc.tcp://IP-Address:4840'}"
ros2 service call /opcua/opcua_client/Subscribe "{node:
  nodeId: 'ns=2;i=2001'
  qualifiedName: 'MyVariable'
callback_topic: 'topic'}"
```
Start echo on created callback topic `topic`:
```
ros2 topic echo /opcuopcua_client/topic
```
and you should get output after few seconds.

### Exmple: Method Call
```
ros2 service call /opcua/opcua_client/Call_method "{node:
  nodeId: 'ns=2;i=99'
  qualifiedName: 'NewObject'
method:
  nodeId: 'ns=2;i=2003'
  qualifiedName: 'MyMethod'
data:
 - {type: '', bool_d: false, int8_d: 0, uint8_d: 0, int16_d: 0, uint16_d: 0, int32_d: 0,
  uint32_d: 0, int64_d: 0, uint64_d: 0, float_d: 0.0, double_d: 0.0, string_d: ''}}"
```
As result you get "MyMethod called!" output in terminal where example server is started.

### Compiling of freeopcua library (Not needed - it is done automatically)

#### Compile freeopcua under ROS2 Foxy

In order to compile freeopcua under Foxy you need to merge [PR356](https://github.com/FreeOpcUa/freeopcua/pull/356) into the freeopuca submodule.

This package implements bindings for freeopcua - Open Source C++ OPC-UA Server and Client Library.

First you need to build FreeOpcUa with following commands (asume that you are in FreeOpcUa folder):

```sh
mkdir build
cd build
cmake ..
make
```

And change `FreeOpcUa_LIBRARIES` variable to be compatible with your environment.

Enjoy!
