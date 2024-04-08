#include "active_vision/debugNode.h"

void displayNodes()
{
  cout << "Available nodes:" << endl;
  for (const pair<int, string> &pair : AVAILABLE_NODES)
  {
    cout << "\t" << pair.first << ": " << pair.second << endl;
  }
  
}

void getUserInput(ros::Publisher debugPub)
{
  bool valid_input = false;
  string user_input;
  string message = "";
  int selected_node;
  int debug_level;
  std_msgs::String msg;
  while (!valid_input)
  {
    cout << "Enter a node's key: ";
    cin >> user_input;
    selected_node = atoi(user_input.c_str());
    if (!AVAILABLE_NODES.count(selected_node))
    {
      cout << user_input << " is not a valid node key, please choose from the following." << endl;
      displayNodes();
      continue;
    }
    cout << "Enter debug level: ";
    cin >> user_input;
    debug_level = atoi(user_input.c_str());
    message = toDebugMessage(AVAILABLE_NODES.at(selected_node), debug_level);
    valid_input = true;
    pair<string, int> output = fromDebugMessage(message);
  }
  msg.data = message;
  debugPub.publish(msg);
  cout << "------------------" << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "DebugDispatcher");
  ros::NodeHandle nh;
  ros::Publisher debugPub = nh.advertise<std_msgs::String>("debugDelta", 1);
  signal(SIGINT, dieWithGrace);
  displayNodes();
  while (ros::ok())
  {
    getUserInput(debugPub);
    ros::spinOnce();
  }
  return (0);
}