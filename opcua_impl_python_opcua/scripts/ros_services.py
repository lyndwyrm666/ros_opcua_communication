#!/usr/bin/env python

# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_service_caller/src/rqt_service_caller/service_caller_widget.py
import math
import numpy
import random
import time

import genpy
import rospy
import rosservice
from opcua import ua, uamethod

import ros_server


class OpcUaROSService:
    def __init__(self, server, parent, idx, service_name, service_class):
        self.server = server
        self.name = service_name
        self.parent = self.recursive_create_objects(service_name, idx, parent)
        self._class = service_class
        self.proxy = rospy.ServiceProxy(self.name, self._class)
        self.counter = 0
        self._nodes = {}
        self.expressions = {}
        self._eval_locals = {}

        for module in (math, random, time):
            self._eval_locals.update(module.__dict__)
        self._eval_locals['genpy'] = genpy
        del self._eval_locals['__name__']
        del self._eval_locals['__doc__']
        # Build the Array of inputs
        sample_req = self._class._request_class()
        sample_resp = self._class._response_class()
        inputs = getargarray(sample_req)
        self.outputs = getargarray(sample_resp)
        self.method = self.parent.add_method(idx, self.name, self.call_service, inputs, self.outputs)

    @uamethod
    def call_service(self, parent, *inputs):
        try:
            # arguments = map(lambda oneinput: oneinput.Value, inputs)
            response = self.proxy(*inputs)
            outputarray = self.filloutputarray(response)
            if not (outputarray is None or len(outputarray) == 0):
                return outputarray
            else:
                return
        except Exception as e:
            print(e)

    def recursive_delete_items(self, item):
        self.proxy.close()
        for child in item.get_children():
            self.recursive_delete_items(child)
            if child in self._nodes:
                del self._nodes[child]
            self.server.delete_nodes([child])
        self.server.delete_nodes([self.method])

    def filloutputarray(self, response):
        print response
        outputs = []
        counter = 0
        for slot_name in response.__slots__:
            slot = getattr(response, slot_name)
            if hasattr(slot, '_type'):
                slot_type = slot._type
                slot_desc = slot._description
                output_arg = ua.Argument()
                output_arg.Name = "Input Argument " + repr(counter)
                output_arg.DataType = ua.NodeId(getobjectidfromtype(slot_type))
                output_arg.ValueRank = -1
                output_arg.ArrayDimensions = []
                output_arg.Description = ua.LocalizedText(slot_desc)

            else:
                if isinstance(slot, list):
                    output_arg = ua.Argument()
                    output_arg.Name = "Input Argument " + repr(counter)
                    output_arg.DataType = ua.NodeId(getobjectidfromtype("array"))
                    output_arg.ValueRank = -1
                    output_arg.ArrayDimensions = []
                    output_arg.Description = ua.LocalizedText("Array")
                else:
                    print("Output Value is a primitive: " + slot_name)

                    output_arg = ua.Argument()
                    output_arg.Name = slot_name
                    output_arg.DataType = ua.NodeId(getobjectidfromtype(type(slot)))
                    output_arg.ValueRank = -1
                    output_arg.ArrayDimensions = []
                    output_arg.Description = ua.LocalizedText(slot_name)
            counter += 1
            if output_arg is not None:
                outputs.append(output_arg.to_binary())
        return outputs

    def recursive_create_objects(self, topic_name, idx, parent):
        hierachy = topic_name.split('/')
        if len(hierachy) == 0 or len(hierachy) == 1:
            return parent
        for name in hierachy:
            if name != '':
                try:
                    nodewithsamename = self.server.get_node(ua.NodeId(name, idx))
                    if nodewithsamename is not None and nodewithsamename.get_parent() == parent:
                        return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx, nodewithsamename)
                    else:
                        newparent = parent.add_object(
                            ua.NodeId(name + str(random.randint(0, 10000)), parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                            ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                        return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx, newparent)
                except Exception:
                    newparent = parent.add_object(
                        ua.NodeId(name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                        ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                    return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx, newparent)
        return parent


def primitivetovariant(typeofprimitive):
    if isinstance(typeofprimitive, list):
        dv = ua.VariantType.Null
    elif typeofprimitive == bool:
        dv = ua.VariantType.Boolean
    elif typeofprimitive == numpy.byte:
        dv = ua.VariantType.Byte
    elif typeofprimitive == int:
        dv = ua.VariantType.Int32
    elif typeofprimitive == float:
        dv = ua.VariantType.Float
    elif typeofprimitive == numpy.double:
        dv = ua.VariantType.Double
    elif typeofprimitive == str:
        dv = ua.VariantType.String
    else:
        # print (typeofprimitive)
        return ua.VariantType.ByteString
    return dv


def getargarray(sample_req):
    array = []
    counter = 0
    for slot_name in sample_req.__slots__:
        slot = getattr(sample_req, slot_name)
        if hasattr(slot, '_type'):
            slot_type = slot._type
            slot_desc = slot._description
            arg = ua.Argument()
            arg.Name = slot_name
            arg.DataType = ua.NodeId(getobjectidfromtype(slot_type))
            arg.ValueRank = -1
            arg.ArrayDimensions = []
            arg.Description = ua.LocalizedText(slot_desc)

        else:
            if isinstance(slot, list):
                slot_type = primitivetovariant("list")
                arg = ua.Argument()
                arg.Name = slot_name
                arg.DataType = ua.NodeId(getobjectidfromtype("array"))
                arg.ValueRank = -1
                arg.ArrayDimensions = []
                arg.Description = ua.LocalizedText("Array")
            else:
                arg = ua.Argument()
                arg.Name = slot_name
                arg.DataType = ua.NodeId(getobjectidfromtype(type(slot).__name__))
                arg.ValueRank = -1
                arg.ArrayDimensions = []
                arg.Description = ua.LocalizedText(slot_name)
        array.append(arg)
        counter += 1

    return array


def refresh_services(server, servicesdict, idx, services_object_opc):
    rosservices = rosservice.get_service_list()

    for service_name_ros in rosservices:
        try:
            if service_name_ros not in servicesdict or servicesdict[service_name_ros] is None:
                service = OpcUaROSService(server, services_object_opc, idx, service_name_ros,
                                          rosservice.get_service_class_by_name(service_name_ros))
                servicesdict[service_name_ros] = service
        except (rosservice.ROSServiceException, rosservice.ROSServiceIOException) as e:
            server.stop()
            print (e)
    tobedeleted = []
    rosservices = rosservice.get_service_list()
    for service_nameOPC in servicesdict:
        found = False
        for rosservice_name in rosservices:
            if service_nameOPC == rosservice_name:
                found = True
        if not found and servicesdict[service_nameOPC] is not None:
            servicesdict[service_nameOPC].recursive_delete_items(server.get_node(ua.NodeId(service_nameOPC, idx)))
            tobedeleted.append(service_nameOPC)
        if len(servicesdict[service_nameOPC].parent.get_children()) == 0:
            server.delete_nodes([servicesdict[service_nameOPC].parent])
    for name in tobedeleted:
        del servicesdict[name]


def getobjectidfromtype(type_name):
    if type_name == 'bool':
        dv = ua.ObjectIds.Boolean
    elif type_name == 'byte':
        dv = ua.ObjectIds.Byte
    elif type_name == 'int8':
        dv = ua.ObjectIds.SByte
    elif type_name == 'uint8':
        dv = ua.ObjectIds.Byte
    elif type_name == 'int16':
        dv = ua.ObjectIds.Int16
    elif type_name == 'uint16':
        dv = ua.ObjectIds.UInt16
    elif type_name == 'int32':
        dv = ua.ObjectIds.Int32
    elif type_name == 'uint32':
        dv = ua.ObjectIds.UInt32
    elif type_name == 'int64':
        dv = ua.ObjectIds.Int64
    elif type_name == 'uint64':
        dv = ua.ObjectIds.UInt64
    elif type_name == 'float' or type_name == 'float32' or type_name == 'float64':
        dv = ua.ObjectIds.Float
    elif type_name == 'double':
        dv = ua.ObjectIds.Double
    elif type_name == 'string' or type_name == 'str':
        dv = ua.ObjectIds.String
    elif type_name == 'array':
        dv = ua.ObjectIds.ArrayItemType
    else:
        # print (type_name)
        return None
    return dv
