#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense> 
#include <vector>      
#include <string>     
#include <thread>
#include <iostream>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
using namespace std;

class hiveMasterNode : public rclcpp::Node
{
public:
  hiveMasterNode() : Node("Hive_MasterNode") 
  {
    // Create publishers for various topics.
    plannerGoal_pub = this->create_publisher<std_msgs::msg::Int64MultiArray>("/plannerGoal", 100);
    serving_pub = this->create_publisher<std_msgs::msg::Int64MultiArray>("/servingGoal", 100);
    parking_pub = this->create_publisher<std_msgs::msg::Int64MultiArray>("/parkingGoal", 100);
    dockingGoal_pub = this->create_publisher<std_msgs::msg::Int16MultiArray>("/dockingGoal", 100);

    // Create subscriptions for various topics and bind callback functions.
    robotState_sub = this->create_subscription<std_msgs::msg::Int16MultiArray>("/stateFromplanner", 100, bind(&hiveMasterNode::robotState_callback, this, placeholders::_1));
    userSelectgoal_sub = this->create_subscription<std_msgs::msg::Int16MultiArray>("/goalFromWeb", 100, bind(&hiveMasterNode::userSelectgoal_callback, this, placeholders::_1));
    serving_sub = this->create_subscription<std_msgs::msg::Int16MultiArray>("/serveFromtable", 100, bind(&hiveMasterNode::serving_callback, this, placeholders::_1));
    parking_sub = this->create_subscription<std_msgs::msg::Int16>("/parkingFromtable", 100, bind(&hiveMasterNode::parking_callback, this, placeholders::_1));

    // Create a timer that calls the run method periodically.
    timer_ = this->create_wall_timer(chrono::milliseconds(100), bind(&hiveMasterNode::run, this));

    // Initialize robot state vector with the state FREE_STATE (1) for each robot.
    for (int i = 0; i < totalRobotcnt; i++)
    {
      robotState_vec.push_back(1);
    }

    std::cout << "robotState_vec" << std::endl;
    for (size_t i = 0; i < robotState_vec.size(); i++)
    {
      std::cout << robotState_vec[i] << " ";
    }
    std::cout << std::endl;
  }

  //====================Callback Functions============================
  void robotState_callback(std_msgs::msg::Int16MultiArray stateFromplanner_msg)
  {
    // Callback for robot state updates
    robotNum = stateFromplanner_msg.data[0];
    robotState_vec[robotNum - 1] = stateFromplanner_msg.data[1];

    std::cout << "robotState_vec" << std::endl;
    for (size_t i = 0; i < robotState_vec.size(); i++)
    {
      std::cout << robotState_vec[i] << " ";
    }
    std::cout << std::endl;
  }

  void userSelectgoal_callback(std_msgs::msg::Int16MultiArray userSelec_msgs)
  {
    // Callback for user-selected goal
    int k = 0;
    webSelect_goal[0] = userSelec_msgs.data[0]; // Desired table count
    webSelect_goal[1] = userSelec_msgs.data[1]; // grid_x (camera coordinate system)
    webSelect_goal[2] = userSelec_msgs.data[2]; // grid_y
    webSelect_goal[3] = userSelec_msgs.data[3]; // grid_yaw
    checkFreetable(totalRobotcnt, webSelect_goal[0]);
  }

  void serving_callback(std_msgs::msg::Int16MultiArray signalFromtable_msg)
  {
    // Callback for serving signal
    std::cout << "serving callback,,," << std::endl;

    if (robotState_vec[signalFromtable_msg.data[0]-1] == TABLE_STATE){
      serving_goal[0] = signalFromtable_msg.data[0]; // Robot number
      serving_goal[1] = signalFromtable_msg.data[1]; // x
      serving_goal[2] = signalFromtable_msg.data[2]; // y
      serving_goal[3] = signalFromtable_msg.data[3]; // yaw
      checkFreeserve(totalRobotcnt, 1);
    }
  }

  void parking_callback(std_msgs::msg::Int16 parkingFromtable_mag)
  {
    // Callback for parking signal
    if (robotState_vec[parkingFromtable_mag.data-1] == TABLE_STATE)
    {
      robotState_vec[parkingFromtable_mag.data-1] = PARKING_STATE;
    }
  }

  // Utility functions
  void checkFreetable(int totalRobotcnt, int desired_cnt)
  {
    // Check if there are enough free tables
    int k = 0; // Counter for free tables
    for (int i = 0; i < totalRobotcnt; i++)
    {
      if (robotState_vec[i] == FREE_STATE)
      {
        k += 1;
        if (k == desired_cnt)
        {
          std::cout << "Desired count reached" << std::endl;
          tableSetting_flag = true;
          return; // Return if the desired count is reached
        }
      }
    }
    // If the loop completes without returning, there are not enough free tables
    std::cout << "There is no Free table" << std::endl;
  }

  void checkFreeserve(int totalRobotcnt, int desired_cnt)
  {
    // Check if there are enough free robots to serve
    int k = 0; // Counter for free robots
    for (int i = 0; i < totalRobotcnt; i++)
    {
      if (robotState_vec[i] == FREE_STATE)
      {
        k += 1;
        if (k == desired_cnt)
        {
          std::cout << "Find free to serve" << std::endl;
          serving_flag = true;
          return; // Return if the desired count is reached
        }
      }
    }
    // If the loop completes without returning, there are not enough free robots
    std::cout << "There is no Free table" << std::endl;
  }

  void printMultiarray(vector<vector<int>> Multivec_)
  {
    // Print a 2D vector
    for (const auto &innerVec : Multivec_)
    {
      for (int num : innerVec)
      {
        std::cout << num << " ";
      }
      std::cout << " " << std::endl; // Print each inner vector on a new line
    }
  }

  void printArray(vector<int> vec_)
  {
    // Print a 1D vector
    for (int num : vec_)
    {
      std::cout << num << " ";
    }
    std::cout << " " << std::endl; // Print a new line after the vector
  }

  vector<int> rotate2D(const vector<int> &point, double angle)
  {
    // Rotate a 2D point by a given angle
    vector<int> rotatedPoint(2);
    rotatedPoint[0] = static_cast<int>(round(point[0] * cos(angle) - point[1] * sin(angle)));
    rotatedPoint[1] = static_cast<int>(round(point[0] * sin(angle) + point[1] * cos(angle)));
    return rotatedPoint;
  }

  vector<vector<int>> moveArrange(vector<int> desiredV)
  {
    // Arrange robots for movement
    int human_num;
    int grid_x, grid_y, yaw_value;

    human_num = webSelect_goal[0]; // Number of robots
    grid_x = webSelect_goal[1]; // x coordinate
    grid_y = webSelect_goal[2]; // y coordinate
    yaw_value = webSelect_goal[3]; // yaw value

    vector<vector<int>> robot_data(human_num, vector<int>(3)); // Initialize vector for robot data

    // Set position and direction of each robot
    for (int i = 0; i < human_num; ++i)
    {
      if (i % 2 == 0)
      {
        robot_data[i][0] = grid_x - gridX_dist * (i / 2.0);
        robot_data[i][1] = grid_y;
        robot_data[i][2] = 1;
      }
      else
      {
        robot_data[i][0] = grid_x - gridX_dist * (i / 2.0);
        robot_data[i][1] = grid_y + gridY_dist;
        robot_data[i][2] = -1;
      }
    }

    if (yaw_value == 2)
    {
      for (int i = 0; i < human_num; ++i)
      {
        // Translate to origin
        robot_data[i][0] -= grid_x;
        robot_data[i][1] -= grid_y;

        // Rotate
        vector<int> rotatedPoint = rotate2D({robot_data[i][0], robot_data[i][1]}, M_PI / 2);
        robot_data[i][0] = rotatedPoint[0];
        robot_data[i][1] = rotatedPoint[1];

        // Adjust yaw angle
        if (robot_data[i][2] == 1)
        {
          robot_data[i][2] = 2;
        }
        else if (robot_data[i][2] == -1)
        {
          robot_data[i][2] = -2;
        }

        // Translate back to original position
        robot_data[i][0] += grid_x;
        robot_data[i][1] += grid_y;
      }
    }
    for (int i = 0; i < human_num; ++i)
    {
      // Scale coordinates
      robot_data[i][0] = (robot_data[i][0]) / batch_size;
      robot_data[i][1] = (robot_data[i][1]) / batch_size;
      std::cout << "Robot " << i + 1 << ": x = " << robot_data[i][0] << ", y = " << robot_data[i][1]
                << ", yaw = " << robot_data[i][2] << std::endl;
    }
    return robot_data;
  }

  vector<vector<int>> tableArrange(vector<int> desiredV)
  {
    // Arrange robots around the table
    int human_num;
    int grid_x, grid_y, yaw_value;

    human_num = webSelect_goal[0]; // Number of robots
    grid_x = webSelect_goal[1]; // x coordinate
    grid_y = webSelect_goal[2]; // y coordinate
    yaw_value = webSelect_goal[3]; // yaw value

    vector<vector<int>> robot_data(human_num, vector<int>(3)); // Initialize vector for robot data

    // Set position and direction of each robot around the table
    for (int i = 0; i < human_num; ++i)
    {
      if (i % 2 == 0)
      {
        robot_data[i][0] = grid_x - (gridX_dist / 4) + ((WHEELBASE / 4) + SAFE_DISTANCE) - (WHEELBASE + 4 * SAFE_DISTANCE) * (i / 2.0);
        robot_data[i][1] = grid_y + (gridY_dist / 2) - (WHEEL_TO_TRIANGLE + (TRIANGLE_h / 2) + SAFE_DISTANCE);
        robot_data[i][2] = 1;
      }
      else
      {
        robot_data[i][0] = grid_x - (gridX_dist / 4) - ((WHEELBASE / 4) + SAFE_DISTANCE) - (WHEELBASE + 4 * SAFE_DISTANCE) * ((i - 1) / 2.0);
        robot_data[i][1] = grid_y + (gridY_dist / 2) + (WHEEL_TO_TRIANGLE + (TRIANGLE_h / 2) + SAFE_DISTANCE);
        robot_data[i][2] = -1;
      }
    }

    if (yaw_value == 2)
    {
      for (int i = 0; i < human_num; ++i)
      {
        // Translate to origin
        robot_data[i][0] -= grid_x;
        robot_data[i][1] -= grid_y;

        // Rotate
        vector<int> rotatedPoint = rotate2D({robot_data[i][0], robot_data[i][1]}, M_PI / 2.0);
        robot_data[i][0] = rotatedPoint[0];
        robot_data[i][1] = rotatedPoint[1];

        // Adjust yaw angle
        if (robot_data[i][2] == 1)
        {
          robot_data[i][2] = 2;
        }
        else if (robot_data[i][2] == -1)
        {
          robot_data[i][2] = -2;
        }

        // Translate back to original position
        robot_data[i][0] += grid_x;
        robot_data[i][1] += grid_y;
      }
    }

    for (int i = 0; i < human_num; ++i)
    {
      // Scale coordinates
      robot_data[i][0] = (robot_data[i][0]) / batch_size;
      robot_data[i][1] = (robot_data[i][1]) / batch_size;
      std::cout << "Robot " << i + 1 << ": x = " << robot_data[i][0] << ", y = " << robot_data[i][1]
                << ", yaw = " << robot_data[i][2] << std::endl;
    }
    return robot_data;
  }

  void tableInitFunction(int totalRobot_cnt, int desired_cnt)
  {
    // Initialize robots to table
    std::cout << "===tableInitFunction" << std::endl;
    plannerPub_vec.clear();
    initState_vec.push_back(vector<int>());
    int k = 0;
    for (int i = 0; i < totalRobot_cnt; i++)
    {
      if (robotState_vec[i] == FREE_STATE)
      {
        k += 1;
        plannerPub_vec.push_back({i + 1, INIT_STATE});
        initState_vec.back().push_back(i + 1);
        robotState_vec[i] = INIT_STATE;

        std::cout << "robotState_vec" << std::endl;
        printArray(robotState_vec);

        if (k == int(desired_cnt))
        {
          std::cout << "initState_vec" << std::endl;
          printMultiarray(initState_vec);
          break;
        }
      }
    }

    std::cout << "INIT_S plannerPub_vec" << std::endl;
    printMultiarray(plannerPub_vec);
    std_msgs::msg::Int64MultiArray plannerPub_;
    for (auto innerVec : plannerPub_vec)
    {
      innerVec.insert(innerVec.end(), 3, 1); // Convert int to int64_t

      // Insert data
      for (int num : innerVec)
      {
        plannerPub_.data.push_back(num); // Convert int to int64_t
      }

      // Publish
      std::cout << "Publish,,," << std::endl;
      plannerGoal_pub->publish(plannerPub_);

      // Wait for a moment to ensure the message is transmitted over the network
      rclcpp::sleep_for(std::chrono::milliseconds(100));

      // Clear the message data
      plannerPub_.data.clear();
    }
    plannerPub_vec.clear();
  }

  void tableMoveFunction(int cnt, vector<int> webmoveStateV)
  {
    // Move robots to table
    std::cout << "===tableMoveFunction" << std::endl;

    std::cout << "plannerPub_vec" << std::endl;
    printMultiarray(plannerPub_vec);

    plannerPub_vec.clear();
    std::cout << "cleared plannerPub_vec" << std::endl;
    printMultiarray(plannerPub_vec);

    vector<vector<int>> mat_vec = moveArrange(webmoveStateV); // Get move arrangement
    p_entry_vec = mat_vec; 

    vector<vector<int>> mat2_vec = tableArrange(webmoveStateV); // Get table arrangement

    // Populate plannerPub_vec with robot number, state, and goal positions
    for (int i = 0; i < mat_vec.size(); ++i)
    {
      vector<int> newRow;
      newRow.push_back(webmoveStateV[i]); // Robot number
      newRow.push_back(WEBMOVE_STATE); // State
      newRow.insert(newRow.end(), mat_vec[i].begin(), mat_vec[i].end()); // Goal positions
      plannerPub_vec.push_back(newRow);
    }
    std::cout << "New plannerPub_vec" << std::endl;
    printMultiarray(plannerPub_vec);

    std_msgs::msg::Int64MultiArray plannerPub_;
    for (auto innerVec : plannerPub_vec)
    {
      // Insert data
      for (int num : innerVec)
      {
        plannerPub_.data.push_back(num); // Convert int to int64_t
      }
      std::cout << "Publish,,," << std::endl;
      plannerGoal_pub->publish(plannerPub_);
      rclcpp::sleep_for(std::chrono::seconds(25)); // Wait for 25 seconds
      plannerPub_.data.clear();
    }

    std_msgs::msg::Int16MultiArray dockingGoal_Pub;
    vector<vector<int>> dockingGoal_vec;

    // Populate dockingGoal_vec with robot number and goal positions
    for (int i = 0; i < mat2_vec.size(); ++i)
    {
      vector<int> newV;
      newV.push_back(webmoveStateV[i]);
      newV.insert(newV.end(), mat2_vec[i].begin(), mat2_vec[i].end());
      dockingGoal_vec.push_back(newV);
    }

    std::cout << "New Table Arrange vec" << std::endl;
    printMultiarray(dockingGoal_vec);

    // Publish docking goals
    for (auto innerVec : dockingGoal_vec)
    {
      for (int num : innerVec)
      {
        dockingGoal_Pub.data.push_back(num);
      }
      std::cout << "Publish,,," << std::endl;
      dockingGoal_pub->publish(dockingGoal_Pub);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      dockingGoal_Pub.data.clear();
    }

    dockingGoal_vec.erase(dockingGoal_vec.end());
  }

  void tableDockingFunction(vector<int> dockingStateV)
  {
    // Dock robots at the table
    std::cout << "===tableDockingFunction" << std::endl;

    std::cout << "plannerPub_vec" << std::endl;
    printMultiarray(plannerPub_vec);
    plannerPub_vec.clear();
    std::cout << "cleared plannerPub_vec" << std::endl;
    printMultiarray(plannerPub_vec);
    vector<vector<int>> mat_vec = {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}}; // Dummy data for docking positions

    // Populate plannerPub_vec with robot number, state, and goal positions
    for (int i = 0; i < mat_vec.size(); ++i)
    {
      vector<int> newRow;
      newRow.push_back(dockingStateV[i]); // Robot number
      newRow.push_back(DOCKING_STATE); // State
      newRow.insert(newRow.end(), mat_vec[i].begin(), mat_vec[i].end()); // Goal positions
      plannerPub_vec.push_back(newRow);
    }
    std::cout << "New plannerPub_vec" << std::endl;
    printMultiarray(plannerPub_vec);

    std_msgs::msg::Int64MultiArray plannerPub_;
    for (auto innerVec : plannerPub_vec)
    {
      // Insert data
      for (int num : innerVec)
      {
        plannerPub_.data.push_back(num); // Convert int to int64_t
      }
      std::cout << "Publish,,," << std::endl;
      plannerGoal_pub->publish(plannerPub_);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      plannerPub_.data.clear();
    }
    plannerPub_vec.erase(plannerPub_vec.end());
  }

  void tableDoneFunction(vector<int> tableStateV)
  {
    // Mark robots as done at the table
    std::cout << "===tableDoneFunction" << std::endl;

    for (int i = 0; i < size(tableStateV); i++)
    {
      robotState_vec[tableStateV[i]] == TABLE_STATE;
      std::cout << "robotState_vec" << std::endl;
      for (size_t i = 0; i < robotState_vec.size(); i++)
      {
        std::cout << robotState_vec[i] << " ";
      }
      std::cout << std::endl;
    }
    // Indicates that all robots have been set to table state.
  }

  void servingFunction(int cnt)
  {
    // Function to handle serving
    std::cout << "===servingFunction" << std::endl;
    std_msgs::msg::Int64MultiArray servingPub_;

    // Find a free robot to serve
    for (int i = 0; i < cnt; i++)
    {
      if (robotState_vec[i] == FREE_STATE)
      {
        servingPub_.data.push_back(i + 1); // Robot number
        servingPub_.data.push_back(SERVING_STATE); // State
        servingState_vec.push_back({i + 1}); // Add robot to serving state vector
        robotState_vec[i] = SERVING_STATE; // Update robot state
        break;
      }
    }

    // Determine the goal position based on the yaw value
    if (serving_goal[3] == 1) 
    {
      std::cout << "yaw == 1" << std::endl;

      servingPub_.data.push_back(serving_goal[1]);
      servingPub_.data.push_back(serving_goal[2] - serving_const);
      servingPub_.data.push_back(serving_goal[3]); // 1, -1 both can be -2
      std::cout << "servingPub_: " << servingPub_.data[0] << ", " << servingPub_.data[1] << ", " << servingPub_.data[2] << ", "
                << servingPub_.data[3] << ", " << servingPub_.data[4] << std::endl;
    }
    else if (serving_goal[3] == -1) 
    {
      std::cout << "yaw == -1" << std::endl;

      servingPub_.data.push_back(serving_goal[1]);
      servingPub_.data.push_back(serving_goal[2] + serving_const);
      servingPub_.data.push_back(serving_goal[3]);
      std::cout << "servingPub_: " << servingPub_.data[0] << ", " << servingPub_.data[1] << ", " << servingPub_.data[2] << ", "
                << servingPub_.data[3] << ", " << servingPub_.data[4] << std::endl;
    }
    else if (serving_goal[3] == 2)
    {
      std::cout << "yaw == 2" << std::endl;

      servingPub_.data.push_back(serving_goal[1] + serving_const);
      servingPub_.data.push_back(serving_goal[2]);
      servingPub_.data.push_back(serving_goal[3]);
      std::cout << "servingPub_: " << servingPub_.data[0] << ", " << servingPub_.data[1] << ", " << servingPub_.data[2] << ", "
                << servingPub_.data[3] << ", " << servingPub_.data[4] << std::endl;
    }
    else if (serving_goal[3] == -2)
    {
      std::cout << "yaw == -2" << std::endl;

      servingPub_.data.push_back(serving_goal[1] - serving_const);
      servingPub_.data.push_back(serving_goal[2]);
      servingPub_.data.push_back(serving_goal[3]);
      std::cout << "servingPub_: " << servingPub_.data[0] << ", " << servingPub_.data[1] << ", " << servingPub_.data[2] << ", "
                << servingPub_.data[3] << ", " << servingPub_.data[4] << std::endl;
    }
    else
    {
      std::cout << "Wrong Yaw input,,," << std::endl;
    }

    // Publish the serving goal to the planner
    std::cout << "Publish,,," << std::endl;
    serving_pub->publish(servingPub_);
    servingPub_.data.clear();
    std::cout << "robotState_vec" << std::endl;
    printArray(robotState_vec);
  }

  void parkingFunction(vector<int> parkingStateV)
  {
    // Function to handle parking
    std::cout << "===parkingFunction" << std::endl;

    std_msgs::msg::Int64MultiArray parkingPub_;
    parkingPub_vec.clear();
    for (int i = 0; i < parkingStateV.size(); ++i)
    {
      vector<int> newRow;
      newRow.push_back(parkingStateV[i]); // Robot number
      newRow.push_back(PARKING_STATE);    // State
      parkingPub_vec.push_back(newRow);   // Add new row to parkingPub_vec
    }
    printMultiarray(parkingPub_vec);
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Publish parking state for each robot
    for (auto innerVec : parkingPub_vec)
    {
      for (int num : innerVec)
      {
        parkingPub_.data.push_back(num); // Convert int to int64_t
      }
      parking_pub->publish(parkingPub_);
      std::cout << "Publish,,," << std::endl;
      rclcpp::sleep_for(std::chrono::seconds(20));
      parkingPub_.data.clear();
      parkingState_vec.clear();
    }
  }

  //============== Main Loop ================
  void run()
  {
    // Main loop for handling robot states and actions

    if (tableSetting_flag)
    {
      tableInitFunction(totalRobotcnt, webSelect_goal[0]);
      tableSetting_flag = false;
    }

    if (serving_flag)
    { 
      servingFunction(totalRobotcnt);
      serving_flag = false;
    }

    if (size(initState_vec) > 0)
    {
      for (int i = 0; i < int(size(initState_vec)); i++)
      {
        for (int j = 0; j < int(size(initState_vec[i])); j++)
        {
          if (robotState_vec[initState_vec[i][j] - 1] == WEBMOVE_STATE)
          {
            k += 1;
            if (k == int(size(initState_vec[i])))
            {
              webmoveState_vec.push_back(initState_vec[i]);
              std::cout << "initState_vec" << std::endl;
              printMultiarray(initState_vec);
              std::cout << "webmoveState_vec" << std::endl;
              printMultiarray(webmoveState_vec);
              initState_vec.erase(initState_vec.begin() + i);
              std::cout << "erased initState_vec" << std::endl;
              printMultiarray(initState_vec);
              tableMoveFunction(totalRobotcnt, webmoveState_vec.back());
            }
          }
        }
        k = 0;
      }
    }

    if (int(size(webmoveState_vec)) > 0)
    {
      for (int i = 0; i < int(size(webmoveState_vec)); i++)
      {
        for (int j = 0; j < int(size(webmoveState_vec[i])); j++)
        {
          if (robotState_vec[webmoveState_vec[i][j] - 1] == DOCKING_STATE)
          {
            k += 1;
            if (k == int(size(webmoveState_vec[i])))
            {
              dockingState_vec.push_back(webmoveState_vec[i]);
              std::cout << "webmoveState_vec" << std::endl;
              printMultiarray(webmoveState_vec);
              std::cout << "dockingState_vec" << std::endl;
              printMultiarray(dockingState_vec);
              webmoveState_vec.erase(webmoveState_vec.begin() + i);
              std::cout << "erased webmoveState_vec" << std::endl;
              printMultiarray(webmoveState_vec);
              tableDockingFunction(dockingState_vec.back());
            }
          }
        }
        k = 0;
      }
    }

    if (int(size(dockingState_vec)) > 0)
    {
      for (int i = 0; i < int(size(dockingState_vec)); i++)
      {
        for (int j = 0; j < int(size(dockingState_vec[i])); j++)
        {
          if (robotState_vec[dockingState_vec[i][j] - 1] == TABLE_STATE)
          {
            k += 1;
            if (k == int(size(dockingState_vec[i])))
            {
              tableState_vec.push_back(dockingState_vec[i]);
              std::cout << "dockingState_vec" << std::endl;
              printMultiarray(dockingState_vec);
              std::cout << "tableState_vec" << std::endl;
              printMultiarray(tableState_vec);
              dockingState_vec.erase(dockingState_vec.begin() + i);
              std::cout << "erased dockingState_vec" << std::endl;
              printMultiarray(dockingState_vec);
              tableDoneFunction(tableState_vec.back());
            }
          }
        }
        k = 0;
      }
    }

    // Handle parking

    if (size(tableState_vec) > 0)
    {
      for (int i = 0; i < size(tableState_vec); i++)
      {
        for (int j = 0; j < size(tableState_vec[i]); j++)
        {
          if (robotState_vec[tableState_vec[i][j] - 1] == PARKING_STATE)
          {
            parkingState_vec.push_back(tableState_vec[i]);
            std::cout << "tableState_vec" << std::endl;
            printMultiarray(tableState_vec);
            std::cout << "parkingState_vec" << std::endl;
            printMultiarray(parkingState_vec);
            tableState_vec.erase(tableState_vec.begin() + i);
            std::cout << "erased tableState_vec" << std::endl;
            printMultiarray(tableState_vec);
            parkingFunction(parkingState_vec.back());
            break;
          }
        }
      }
    }

    if (size(servingState_vec) > 0)
    {
      for (int i = 0; i < size(servingState_vec); i++)
      {
        for (int j = 0; j < size(servingState_vec[i]); j++)
        {
          if (robotState_vec[servingState_vec[i][j] - 1] == PARKING_STATE)
          {
            parkingState_vec.push_back(servingState_vec[i]);
            std::cout << "servingState_vec" << std::endl;
            printMultiarray(servingState_vec);
            std::cout << "parkingState_vec" << std::endl;
            printMultiarray(parkingState_vec);
            servingState_vec.erase(servingState_vec.begin() + i);
            std::cout << "erased servingState_vec" << std::endl;
            printMultiarray(servingState_vec);
            parkingFunction(parkingState_vec.back());
          }
        }
      }
    }
  }

private:
  // Constants for real-world dimensions and grid calculations
  float REAL_X = 2.57;
  float REAL_Y = 1.94;
  float fixelConst_x = REAL_X / 640; //[One pixel distance: 0.00401562m] 
  float fixelConst_y = REAL_Y / 480; //[One pixel distance: 0.00404167m]
  int gridY_dist = 210; // 85cm
  int gridX_dist = 270; // 1.08m
  int batch_size = 4;

  // Dimensions for robot calculations
  float TRIANGLE_h = 0.13 / fixelConst_x;        // Size of the top triangle part of the robot
  float WHEEL_TO_TRIANGLE = 0.13 / fixelConst_x; // Distance from wheel center to the triangle part
  float SAFE_DISTANCE = 0.02 / fixelConst_y;      // Safe distance
  float WHEELBASE = 0.436 / fixelConst_y;
  int serving_const = 68; // Constant for serving distance

  // State flags
  bool tableSetting_flag;
  bool serving_flag;
  bool parking_flag;

  // Robot states
  int FREE_STATE = 1;
  int INIT_STATE = 2;
  int WEBMOVE_STATE = 3;
  int DOCKING_STATE = 4;
  int TABLE_STATE = 5;
  int SERVING_STATE = 6;
  int PARKING_STATE = 7;

  // Robot and goal information
  int totalRobotcnt = 3;
  int robotNum;
  int k = 0;

  int webSelect_goal[4] = {0, 0, 0, 0}; // Desired table count, grid_x, grid_y, grid_yaw
  int serving_goal[4] = {0, 0, 0, 0};

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr plannerGoal_pub;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr serving_pub;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr parking_pub;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr dockingGoal_pub;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr robotState_sub;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr userSelectgoal_sub;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr serving_sub;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr parking_sub;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Vectors for robot state and actions
  vector<int> robotState_vec = {};
  vector<vector<int>> freeState_vec;
  vector<vector<int>> initState_vec;
  vector<vector<int>> webmoveState_vec;
  vector<vector<int>> dockingState_vec;
  vector<vector<int>> tableState_vec;
  vector<vector<int>> servingState_vec;
  vector<vector<int>> parkingState_vec;
  vector<vector<int>> plannerPub_vec;
  vector<vector<int>> parkingPub_vec;
  vector<int> servingPub_vec = {};
  vector<int> servePub_vec;
  vector<vector<int>> p_entry_vec;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); // Initialize ROS2
  auto master_node = make_shared<hiveMasterNode>(); // Create the master node

  RCLCPP_INFO(master_node->get_logger(), "Master Node Start!"); // Log message
  thread run_thread([&]()
                    { master_node->run(); }); // Create a thread to run the node
  run_thread.detach();

  rclcpp::spin(master_node); // Spin the node

  rclcpp::shutdown(); // Shutdown ROS2

  return 0;
}
