/** @file
 * @brief       This file implements OpcUa client based on FreeOpcUa implementation.
 * @author      Denis Štogl
 *
 * @copyright   Copyright 2015 SkillPro Consortium
 *
 *              This file is part of SkillPro-Framework.
 *
 *              SkillPro-Framework is free software: you can redistribute it and/or modify
 *              it under the terms of the GNU Lesser General Public License as published by
 *              the Free Software Foundation, either version 3 of the License, or
 *              (at your option) any later version.
 *
 *              SkillPro-Framework is distributed in the hope that it will be useful,
 *              but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *              GNU Lesser General Public License for more details.
 *
 *              You should have received a copy of the GNU Lesser General Public License
 *              along with SkillPro-Framework. If not, see <http://www.gnu.org/licenses/>.
 *
 * @todo        Would be cool to have dignostic topic to write subscriptions and stuff
 */

#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <opcua_helpers.h>

#include "ros_opcua_msgs/msg/address.hpp"
#include "ros_opcua_msgs/msg/type_value.hpp"

#include "ros_opcua_srvs/srv/call_method.hpp"
#include "ros_opcua_srvs/srv/connect.hpp"
#include "ros_opcua_srvs/srv/disconnect.hpp"
#include "ros_opcua_srvs/srv/list_node.hpp"
#include "ros_opcua_srvs/srv/read_opc.hpp"
#include "ros_opcua_srvs/srv/subscribe.hpp"
#include "ros_opcua_srvs/srv/write_opc.hpp"
#include "ros_opcua_srvs/srv/unsubscribe.hpp"

#include <opc/ua/client/client.h>
#include <opc/ua/subscription.h>

/// Client variable
OpcUa::UaClient _client(false);
/// Subscription list
/** Key is OpcUa string node identifier and value subscription reference.*/
std::map<std::string, std::shared_ptr<OpcUa::Subscription>> _subscriptions;
/// Callback publishers list
/** Key is OpcUa string node identifier and value ROS publisher */
std::map<std::string, std::shared_ptr<rclcpp::Publisher<ros_opcua_msgs::msg::TypeValue, std::allocator<void>>>> _callback_publishers;
/// List of subscription handles
/** Key is OpcUa string node identifier and value is node handle. This is needed for deteting of subscriptions*/
std::map<std::string, uint32_t> _subscription_handles;
/// Connected status variable
bool _connected = false;

/// Subscription client class handles subscription callbacks.
/**
 * Currently is only implemented reaction on data change.
 */
class SubClient : public OpcUa::SubscriptionHandler
{
  /**
   * DataChange function is called by client internal mechanisms when change of data on subscribed node.
   * @param handle subscription handle
   * @param node node on which data were changed
   * @param value new value
   * @param attr OpcUa Attribute Id
   */
  void DataChange(uint32_t handle, const OpcUa::Node& node, const
OpcUa::Variant& value, OpcUa::AttributeId attr) override
  {
    //ROS_DEBUG("Callback....");
    _callback_publishers[OpcUa::ToString(node.GetId())]->publish(convertVariantToTypeValue(value));
  }
};

/// Subscription client store
/** @todo Need checking for multiple subscriptions */
SubClient _sclt;

/**
 * ROS-service call: Connect. Connects to OpcUa server.
 * @param req service request
 * @param res service response
 * @return True, if service execution broken than False.
 */
void connect(std::shared_ptr<ros_opcua_srvs::srv::Connect::Request> req, std::shared_ptr<ros_opcua_srvs::srv::Connect::Response> res)
{
    // TODO: set connect status

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Establishing connection to OPC-UA server on address: %s", req->endpoint.c_str());
    try {
        _client.Connect(req->endpoint);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Connection to OPC-UA server on address '%s' established!", req->endpoint.c_str());
        res->success = true;
        _connected = true;
    }
    catch (const std::exception& exc){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OPC-UA client node: Connection to OPC-UA server on address %s failed! Message: %s",req->endpoint.c_str(), exc.what());

        res->success = false;
        std::stringstream ss;
        ss << "Connection to OPC-UA server on address " << req->endpoint << " failed! Message: " << exc.what();
        res->error_message = ss.str();
    }
    catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Connection to OPC-UA server on address %s failed with unknown exception", req->endpoint.c_str());
        res->success = false;
        res->error_message = "'Connect' service failed with Unknown exception";
    }
}

/**
 * ROS-service call: Disconnect. Disconnects from OpcUa server.
 * @param req service request
 * @param res service response
 * @return True, if service execution broken than False.
 */
void disconnect(std::shared_ptr<ros_opcua_srvs::srv::Disconnect::Request> req, std::shared_ptr<ros_opcua_srvs::srv::Disconnect::Response> res)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Disconnecting from OPC-UA server...");
    try {
        _client.Disconnect();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Disconnection succeded!");
        _connected = false;
        res->success = true;
    }
    catch (const std::exception& exc){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OPC-UA client node: Disconnection failed! (maybe client was not connected before?). Message: %s", exc.what());

        res->success = false;
        res->error_message = "Disconect service failed with exception: ";
        res->error_message.append(exc.what());
    }
    catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"'Disconnect' service failed with unknown exception");
        res->success = false;
        res->error_message = "'Disconnect' service failed with unknown exception";
    }
}

/**
 * ROS-service call: ListNode. Gets list of Node children from OpcUa server.
 * @param req service request
 * @param res service response
 * @return True, if service execution broken than False.
 */
void list_node(std::shared_ptr<ros_opcua_srvs::srv::ListNode::Request> req, std::shared_ptr<ros_opcua_srvs::srv::ListNode::Response> res)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'ListNode' service called with node Id: %s parameters", req->node.nodeid.c_str());

    OpcUa::Node node;
    try {
      if (req->node.nodeid == "") {
          node = _client.GetObjectsNode();
      }
      else {
        node = _client.GetNode(req->node.nodeid);
      }

      for (OpcUa::Node child : node.GetChildren()){
          ros_opcua_msgs::msg::Address address;
          address.nodeid = OpcUa::ToString(child.GetId());
          address.qualifiedname = child.GetBrowseName().Name;
          res->children.push_back(address);
      }
      res->success = true;
    }
    catch (const std::exception& exc){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'ListNode' service called with node Id: %s, parameters failed! Exception: %s", req->node.nodeid.c_str(), exc.what());

            res->success = false;
            res->error_message = "ListNode service failed with exception: ";
            res->error_message.append(exc.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'ListNode' service called with node Id: %s parameters failed! Unknown exception!", req->node.nodeid.c_str());

            res->success = false;
            res->error_message = "ListNode service failed with Unknown exception";
        }
    //    return true;
}

void call_method(std::shared_ptr<ros_opcua_srvs::srv::CallMethod::Request> req, std::shared_ptr<ros_opcua_srvs::srv::CallMethod::Response> res)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'CallMethod' service called with node Id: %s and %s parameters", req->node.nodeid.c_str(), req->method.nodeid.c_str());

    try {
        OpcUa::Node node = _client.GetNode(req->node.nodeid);
        OpcUa::NodeId method_id = OpcUa::ToNodeId(req->method.nodeid);
        std::vector<OpcUa::Variant> inputArguments;

        for (ros_opcua_msgs::msg::TypeValue typeValue : req->data) {
            std::cout << "Variant: " <<
convertTypeValueToVariant(typeValue).ToString() << std::endl;
            inputArguments.push_back(convertTypeValueToVariant(typeValue));
        }

        std::vector<OpcUa::Variant> outputArguments = node.CallMethod(method_id, inputArguments);

        for (OpcUa::Variant variant : outputArguments) {
            res->data.push_back(convertVariantToTypeValue(variant));
        }

        res->success = true;

    }
    catch (const std::exception& exc){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'CallMethod' service called with node Id: %s and %s parameters failed! Exception: %s", req->node.nodeid.c_str(), req->method.nodeid.c_str(), exc.what());

        res->success = false;
        res->error_message = "Call method service failed with exception: ";
        res->error_message.append(exc.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'CallMethod' service called with node Id: %s parameters failed! Unknown exception!", req->node.nodeid.c_str());

        res->success = false;
        res->error_message = "Call method service failed with Unknown exception";
    }
    //return true;
}

/**
 * ROS-service call: Read. Reads value from OpcUa server.
 * @param req service request
 * @param res service response
 * @return True, if service execution broken than False.
 */
void read_opc(std::shared_ptr<ros_opcua_srvs::srv::ReadOpc::Request> req, std::shared_ptr<ros_opcua_srvs::srv::ReadOpc::Response> res)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Read' service called with node Id: %s parameters", req->node.nodeid.c_str());

    try {
        OpcUa::Node variable = _client.GetNode(req->node.nodeid);

        res->success = true;
        res->data = convertVariantToTypeValue(variable.GetValue());

        if (res->data.type == "Unknown") {
            res->success = false;
            res->error_message = "Unknown data type!!";
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Reading failed!");
        }
    }
    catch (const std::exception& exc){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Read' service called with node Id: %s, parameters failed! Exception: %s", req->node.nodeid.c_str(), exc.what());

        res->success = false;
        res->error_message = "Read service failed with exception: ";
        res->error_message.append(exc.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Read' service called with node Id: %s parameters failed! Unknown exception!", req->node.nodeid.c_str());

        res->success = false;
        res->error_message = "Read service failed with Unknown exception";
    }
    //return true;
}

/**
 * ROS-service call: Write. Writes value to OpcUa server.
 * @param req service request
 * @param res service response
 * @return True, if service execution broken than False.
 */
void write_opc(std::shared_ptr<ros_opcua_srvs::srv::WriteOpc::Request> req, std::shared_ptr<ros_opcua_srvs::srv::WriteOpc::Response> res)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Write' service called with node Id: %s, parameters", req->node.nodeid.c_str());

    try {
        OpcUa::Node variable = _client.GetNode(req->node.nodeid);

        OpcUa::Variant variant = convertTypeValueToVariant(req->data);

        if (!variant.IsNul()) {
//           if (variant.Type() == OpcUa::VariantType::STRING) {
//             if (((std::string)variant).length() == 0) {
//               variant = std::string(" ");
//             }
//           }
          variable.SetValue(variant);
          res->success = true;
        }
        else {
            res->success = false;
            res->error_message = "Unknown data type: ";
            res->error_message.append(req->data.type);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Writing failed!");
        }
    }
    catch (const std::exception& exc){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Write' service called with node Id: %s, type: '%s' parameters failed! Exception: %s", req->node.nodeid.c_str(), req->data.type.c_str(), exc.what());

        res->success = false;
        res->error_message = "Write service failed with exception: ";
        res->error_message.append(exc.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Write' service called with node Id: %s, type: '%s' parameters failed! Unknown exception!", req->node.nodeid.c_str(), req->data.type.c_str());

        res->success = false;
        res->error_message = "Write service failed with Unknown exception";
    }

    //return true;
}

/**
 * ROS-service call: Subscribe. Subscribe to DataChange of Node on OpcUa server.
 * @param req service request
 * @param res service response
 * @return True, if service execution broken than False.
 */
/** @todo can subscribe on only one node */
void subscribe(std::shared_ptr<ros_opcua_srvs::srv::Subscribe::Request> req, std::shared_ptr<ros_opcua_srvs::srv::Subscribe::Response> res)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Subscribe' service called with node Id: %s parameters", req->node.nodeid.c_str());

    try {
        // TODO: Check if already subscribed to node

        OpcUa::Node variable = _client.GetNode(req->node.nodeid);
        std::string node_string = OpcUa::ToString(variable.GetId());

        auto nodeHandle = rclcpp::Node::make_shared("~");
        _callback_publishers[node_string] =  nodeHandle->create_publisher<ros_opcua_msgs::msg::TypeValue>(req->callback_topic, 1);

        _subscriptions[node_string] = _client.CreateSubscription(100, _sclt);
        _subscription_handles[node_string] =  _subscriptions[node_string]->SubscribeDataChange(variable);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Node successfully subscribed!");
        res->success = true;
    }
    catch (const std::exception& exc){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Subscribe' service called with node Id: %s parameters failed!", req->node.nodeid.c_str());

        res->success = false;
        res->error_message = "Subscribe service failed with exception: ";
        res->error_message.append(exc.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Subscribe' service called with node Id: %s parameters failed!", req->node.nodeid.c_str());

        res->success = false;
        res->error_message = "Subscribe service failed with Unknown exception";
    }

    //return true;
}

/**
 * ROS-service call: Unsubscribe. Unsubscribe from Node on OpcUa server.
 * @param req service request
 * @param res service response
 * @return True, if service execution broken than False.
 */
void unsubscribe(std::shared_ptr<ros_opcua_srvs::srv::Unsubscribe::Request> req, std::shared_ptr<ros_opcua_srvs::srv::Unsubscribe::Response> res)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Unsubscribe' service called with node Id: %s parameters", req->node.nodeid.c_str());

    try {
        // TODO: Check if already subscribed to node

        // TODO: Why is here return status false even the subscription was ok?

        OpcUa::Node variable = _client.GetNode(req->node.nodeid);
        std::string node_string = OpcUa::ToString(variable.GetId());

        _subscriptions[node_string]->UnSubscribe(_subscription_handles[node_string]);

        _subscriptions.erase(node_string);
        _subscription_handles.erase(node_string);
        _callback_publishers.erase(node_string);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Node successfully unsubscribed!");
        res->success = true;
    }
    catch (const std::exception& exc){
       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Unsubscribe' service called with node Id: %s parameters failed!", req->node.nodeid.c_str());

        res->success = false;
        res->error_message = "Unsubscribe service failed with exception: ";
        res->error_message.append(exc.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Unsubscribe' service called with node Id: %s parameters failed!", req->node.nodeid.c_str());

        res->success = false;
        res->error_message = "Unsubscribe service failed with Unknown exception";
    }

    //return true;
}

/**
 * Handler for Shutdown of the node
 */
void on_shutdown(int sig)
{
  _client.Disconnect();

  rclcpp::shutdown();
}

/**
 * Start function of ROS-Node with default name 'opcua_client_node'
 * @param argc number of command line arguments
 * @param argv command line arguments
 * @return 0 if programs ends successfully.
 */
int main (int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> nodeHandle = rclcpp::Node::make_shared("opcua_client");
    signal(SIGINT, on_shutdown);

    //Connect
    rclcpp::Service<ros_opcua_srvs::srv::Connect>::SharedPtr connect_service =
    nodeHandle->create_service<ros_opcua_srvs::srv::Connect>(std::string(nodeHandle->get_name()) + "/connect", &connect);
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Connect' service available on on: %s", connect_service->get_service_name());
    
    //Disconnect
    rclcpp::Service<ros_opcua_srvs::srv::Disconnect>::SharedPtr disconnect_service =
    nodeHandle->create_service<ros_opcua_srvs::srv::Disconnect>(std::string(nodeHandle->get_name()) + "/disconnect", &disconnect);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Disconnect' service available on: %s", disconnect_service->get_service_name());

    // List node
    rclcpp::Service<ros_opcua_srvs::srv::ListNode>::SharedPtr list_service =
    nodeHandle->create_service<ros_opcua_srvs::srv::ListNode>(std::string(nodeHandle->get_name()) + "/list", &list_node);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'ListNode' service available on on: %s", list_service->get_service_name());

    // Method Call
    rclcpp::Service<ros_opcua_srvs::srv::CallMethod>::SharedPtr call_service =
    nodeHandle->create_service<ros_opcua_srvs::srv::CallMethod>(std::string(nodeHandle->get_name()) + "/call_method", &call_method);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'CallMethod' service available on: %s", call_service->get_service_name());

    // Reading of data
    rclcpp::Service<ros_opcua_srvs::srv::ReadOpc>::SharedPtr read_service =
    nodeHandle->create_service<ros_opcua_srvs::srv::ReadOpc>(std::string(nodeHandle->get_name()) + "/read", &read_opc);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Read' service available on: %s", read_service->get_service_name());

    // Writing of data
    rclcpp::Service<ros_opcua_srvs::srv::WriteOpc>::SharedPtr write_service =
    nodeHandle->create_service<ros_opcua_srvs::srv::WriteOpc>(std::string(nodeHandle->get_name()) + "/write", &write_opc);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Write' service available on: %s", write_service->get_service_name());

    // Subscriptions
    rclcpp::Service<ros_opcua_srvs::srv::Subscribe>::SharedPtr subscribe_service =
    nodeHandle->create_service<ros_opcua_srvs::srv::Subscribe>(std::string(nodeHandle->get_name()) + "/subscribe", &subscribe);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Subscribe' service available on: %s", subscribe_service->get_service_name());
    rclcpp::Service<ros_opcua_srvs::srv::Unsubscribe>::SharedPtr unsubscribe_service =
    nodeHandle->create_service<ros_opcua_srvs::srv::Unsubscribe>(std::string(nodeHandle->get_name()) + "/unsubscribe", &unsubscribe);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OPC-UA client node: 'Unsubscribe' service available on: %s", unsubscribe_service->get_service_name());


    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OPCUA client node %s is ready!", nodeHandle->get_name());

    rclcpp::spin(nodeHandle);

    return 0;
}
