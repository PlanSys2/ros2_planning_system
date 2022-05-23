# BT Actions

The purpose of this package is to provide built-in support for `plansys2` actions which use [Behavior Trees](https://github.com/BehaviorTree/BehaviorTree.CPP) (BTs) for their implementation.  A drop-in replacement for a vanilla `plansys2::ActionExecutorClient` is provided in the form of a ROS2 node, which will execute an arbitrary BT passed in as a ROS parameter.  A BehaviorTree.CPP node, `plansys2::BtActionNode`, is also included to provide a convenient wrapper around a ROS2 action client (a common application for BT nodes).

### ROS2 node for implementing `plansys2` actions
The `bt_action_node` ROS node takes in several parameters to set up its execution.
These parameters are
1. `action_name`: The name of the `plansys2` action to implement. Note that this name must match what is in your pddl domain file.
2. `bt_xml_file`: An absolute path to the BT `.xml` file to execute.
3. `plugins`: a list of BehaviorTree.CPP shared libraries to load. Any BT node which is in the `.xml` but is not provided by the BehaviorTree.CPP library itself must be in one of the libraries specified
4. `enable_groot_monitoring`: a boolean which specifies if ZMQ publisher should be created, for use with [Groot](https://github.com/BehaviorTree/Groot) (default is `false`)
5. `publisher_port`: the ZMQ publisher port to use (if `enable_groot_monitoring` is enabled)
6. `server_port`: the ZMQ server port to use (if `enable_groot_monitoring` is enabled)
7. `max_msgs_per_second`: max ZMQ messages per second (if `enable_groot_monitoring` is enabled)
8. `bt_file_logging`: a boolean which [enables logging of BT state changes in `.fbl` files](https://www.behaviortree.dev/tutorial_05_subtrees/), useful for playing back behavior tree execution using `Groot` (default is `false`)
9. `bt_minitrace_logging`: a boolean which enables logging of `.json` files for recording the execution time of each node (default is `false`)

Files created by the `.fbl` and minitrace loggers are stored in `/tmp/<node_name>/`, with names containing a timestamp.

### BT node for calling ROS2 action servers
The `BtActionNode` template class provides a convenient means of calling ROS2 action servers from within a BT.  It takes care of the details of setting up and handling a ROS action client, reducing code duplication and providing a simple API.

The template parameter for the class is the type of ROS action (e.g. `action_tutorials_interfaces::action::Fibonacci`) to be used. The node's constructor takes in three arguments: the XML tag name, the ROS topic for the action server (e.g. `/namepace/server_name`), and a `BT::NodeConfiguration`.  Note that the XML name and `NodeConfiguration` are the same as any other BT.CPP node.

There are several functions which are provided for the end user to use/implement (some of which are optional).
1. `static BT::PortsList providedPorts()`: every BT node which uses ports must define this member function.  A default implementation is provided, but you are free to override it if additional ports are desired.  By default, the function returns two input ports: `server_name` (string) and `server_timeout` (double).  These ports can be preserved when overriding using `providedBasicPorts`
  * `server_name`: an (optional) means of overriding the action server topic provided in the constructor
  * `server_timeout`: how long to wait for an action server before failing, in units of seconds (default is 5s)
2. `BT::PortsList providedBasicPorts(BT::PortsList addition)`: a convenience function for preserving the default input ports when overriding the `providedPorts()` function.  An example use is shown below
  ```cpp
  static BT::PortsList providedPorts() override
  {
    return providedBasicPorts({ BT::InputPort<std::string>("my_additional_port") });
  }
  ```
3. `virtual BT::NodeStatus on_tick()`: This function is called every time the node is ticked (in the BT sense).  The user is expected to set the `goal_` variable somewhere in this function, which is of the same type as the template parameter.  Note that the node will likely be ticked multiple times before the action completes, so you may want to only set the goal on the first tick (i.e. when the node is IDLE).  If you want to preempt the current goal and send a new one after the first tick, set the `goal_updated_` member variable to true, along with any changes to the `goal_` variable.  An example is shown below.
  ```cpp
  BT::NodeStatus on_tick() override
  {
    if (status() == BT::NodeStatus::IDLE) { // this block will only execute on the first tick
      goal_.string = "foo";
    } else if (<some condition>) {  // this block may execute anytime afterward
      goal_.string = "bar";
      goal_updated_ = true;
    }
  }
  ```
4. `virtual void on_feedback(const std::shared_ptr<ActionT::Feedback> feedback)`: this function will be called whenever feedback is received from the action server **asynchronous of BT execution** (and is optional)
5. `virtual BT::NodeStatus on_success()`: this function is called if the action server returned successfully and will return `BT::NodeStatus::SUCCESS` by default.  You may wish to override it if the node's success depends on the result, which is placed in the `result_` member variable.
6. `virtual BT::NodeStatus on_aborted()`: this function is called if the action server aborted the action, returning `BT::NodeStatus::FAILURE` by default.
7. `virtual BT::NodeStatus on_cancelled()`: this function is called if the action was cancelled, returning `BT::NodeStatus::SUCCESS` by default.

