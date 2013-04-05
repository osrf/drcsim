#include <atlas_msgs/AtlasSimInterfaceAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>

typedef actionlib::SimpleActionClient<atlas_msgs::AtlasSimInterfaceAction> Client;
/*
int main(int argc, char** argv)
{
  ros::init(argc, argv, "atlas_client");
  Client client("atlas/bdi_control", true); // true -> don't need ros::spin()

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  client.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action

  atlas_msgs::AtlasSimInterfaceGoal goal;
  // Fill in goal here

  // stand prep
  goal.params.behavior = 4;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas stand prep action suceeded");

  ros::Duration(5).sleep();

  // stand
  goal.params.behavior = 7;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas stand action suceeded");

  ros::Duration(1).sleep();

  // demo1 - walk
  goal.params.behavior = 9;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The altas demo1 action suceeded");

  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}*/


#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45

#define NUM_MULTISTEP_WALK_STEPS 4

class TeleopAtlasKeyboard
{
  private:
  double walk_vel, run_vel, yaw_rate, yaw_rate_run;
  //geometry_msgs::Twist cmd;

  ros::NodeHandle n_;
  ros::Publisher vel_pub_;
  Client *client;
  double strideSagittal;
  double strideCoronal;
  double strideDuration;
  double turnAngle;
  atlas_msgs::AtlasSimInterfaceState currentState;
  atlas_msgs::AtlasBehaviorMultiStepParams multistep;


  public:
  void init()
  {
    this->strideSagittal = 0.23;
    this->strideCoronal = 0.12;
    this->strideDuration = 0.63;
    this->turnAngle = M_PI / 8.0;
    //cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

    //vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::NodeHandle n_private("~");
    n_private.param("walk_vel", walk_vel, 0.5);
    n_private.param("run_vel", run_vel, 1.0);
    n_private.param("yaw_rate", yaw_rate, 1.0);
    n_private.param("yaw_run_rate", yaw_rate_run, 1.5);

//    Client client("atlas/bdi_control", true); // true -> don't need ros::spin()
    // true -> don't need ros::spin()
    client =  new Client("atlas/bdi_control", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    client->waitForServer();
    ROS_INFO("Action server started");

    atlas_msgs::AtlasSimInterfaceGoal goal;
    // stand prep
    goal.params.behavior = 4;
    client->sendGoal(goal);
    client->waitForResult(ros::Duration(5.0));
    if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      printf("The altas stand prep action suceeded");

    ros::Duration(5).sleep();

    // stand
    goal.params.behavior = 7;
    client->sendGoal(goal);
    client->waitForResult(ros::Duration(5.0));
    if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      printf("The altas stand action suceeded");

    ROS_INFO("Atlas ready to walk");

  }

  ~TeleopAtlasKeyboard()
  {
    delete this->client;
  }
  void keyboardLoop();
  void feedbackCB(
      const atlas_msgs::AtlasSimInterfaceFeedbackConstPtr &feedback);
  void resultCB(const actionlib::SimpleClientGoalState& state,
    const atlas_msgs::AtlasSimInterfaceResultConstPtr &result);

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "atlas_base_keyboard");

  TeleopAtlasKeyboard tpk;
  tpk.init();

  signal(SIGINT,quit);

  tpk.keyboardLoop();

  return(0);
}

void TeleopAtlasKeyboard::feedbackCB(
    const atlas_msgs::AtlasSimInterfaceFeedbackConstPtr &feedback)
{
  std::cout <<"fb ec " << feedback->state.error_code << std::endl;
  currentState = feedback->state;
}

void TeleopAtlasKeyboard::resultCB(const actionlib::SimpleClientGoalState& state,
    const atlas_msgs::AtlasSimInterfaceResultConstPtr &result)
{
//  std::cout <<"fb result " << result->state.error_code << std::endl;
//  std::cout <<"fb pelvis pos " << feedback.pelvis_position.x() << std::endl;
}

void TeleopAtlasKeyboard::keyboardLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WASD' to translate");
  puts("Use 'QE' to yaw");
  puts("Press 'Shift' to run");


  for(;;)
  {
    if (this->client->getState()
        == actionlib::SimpleClientGoalState::ACTIVE)
      continue;

    atlas_msgs::AtlasSimInterfaceGoal goal;
    goal.params.behavior = atlas_msgs::AtlasSimInterface::MULTI_STEP_WALK;

    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    //cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

    switch(c)
    {
      // Walking
    case KEYCODE_W:
    {
      std::cout << " forward " << std::endl;
      // for testing: generate 4 steps to match internal atlas step buffer size
      for ( int stepId = 0; stepId < NUM_MULTISTEP_WALK_STEPS; ++stepId)
      {
        multistep.multistep_params[stepId].step_index = stepId + 1;
        multistep.multistep_params[stepId].foot_index = stepId ^ 1;
        multistep.multistep_params[stepId].duration = this->strideDuration;

        double stepX =
          static_cast<double>(stepId + 1) * this->strideSagittal;
        double stepY = this->strideCoronal;
        if ((stepId ^ 1) == 1)
          stepY *= -1;

        geometry_msgs::Pose pose;
        pose.position.x = stepX;
        pose.position.y = stepY;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        multistep.multistep_params[stepId].pose = pose;
      }
      goal.params.multistep_walk_params = multistep;

      dirty = true;
      break;
    }
    case KEYCODE_S:
    {
      std::cout << " backward " << std::endl;
      goal.params.behavior = 5;

      // for testing: generate 4 steps to match internal atlas step buffer size
      for ( int stepId = 0; stepId < NUM_MULTISTEP_WALK_STEPS; ++stepId)
      {
        multistep.multistep_params[stepId].step_index = stepId + 1;
        multistep.multistep_params[stepId].foot_index = stepId ^ 1;
        multistep.multistep_params[stepId].duration = this->strideDuration;

        double stepX =
          static_cast<double>(stepId + 1) * this->strideSagittal * -1;
        double stepY = this->strideCoronal;
        if ((stepId ^ 1) == 1)
          stepY *= -1;

        geometry_msgs::Pose pose;
        pose.position.x = stepX;
        pose.position.y = stepY;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        multistep.multistep_params[stepId].pose = pose;
      }
      goal.params.multistep_walk_params = multistep;

      dirty = true;
      break;
    }
    case KEYCODE_A:
    {
      std::cout << " left " << std::endl;

      // for testing: generate 4 steps to match internal atlas step buffer size
      for ( int stepId = 0; stepId < NUM_MULTISTEP_WALK_STEPS; ++stepId)
      {
        multistep.multistep_params[stepId].step_index = stepId + 1;
        multistep.multistep_params[stepId].foot_index = stepId ^ 1;
        multistep.multistep_params[stepId].duration = this->strideDuration;

        double stepX = 0;
        double stepY = this->strideSagittal
            + static_cast<double>(stepId + 1)*this->strideSagittal/2.0;
        if ((stepId ^ 1) == 1)
          stepY -= this->strideSagittal*2;

        geometry_msgs::Pose pose;
        pose.position.x = stepX;
        pose.position.y = stepY;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        multistep.multistep_params[stepId].pose = pose;
      }
      goal.params.multistep_walk_params = multistep;
      dirty = true;
      break;
    }
    case KEYCODE_D:
    {
      std::cout << " right " << std::endl;

      goal.params.behavior = 5;

      // for testing: generate 4 steps to match internal atlas step buffer size
      for ( int stepId = 0; stepId < NUM_MULTISTEP_WALK_STEPS; ++stepId)
      {
        multistep.multistep_params[stepId].step_index = stepId + 1;
        multistep.multistep_params[stepId].foot_index = stepId ^ 1;
        multistep.multistep_params[stepId].duration = this->strideDuration;

        double stepX = 0;
        double stepY = this->strideSagittal
            - static_cast<double>(stepId + 1)*this->strideSagittal/2.0;
        if ((stepId ^ 1) == 1)
          stepY -= this->strideSagittal*2;

        geometry_msgs::Pose pose;
        pose.position.x = stepX;
        pose.position.y = stepY;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        multistep.multistep_params[stepId].pose = pose;
      }
      goal.params.multistep_walk_params = multistep;
      dirty = true;
      break;
    }
    case KEYCODE_Q:
    {
      std::cout << " turn left " << std::endl;

      double angle = 0;
      // for testing: generate 4 steps to match internal atlas step buffer size
      for ( int stepId = 0; stepId < NUM_MULTISTEP_WALK_STEPS; ++stepId)
      {
        multistep.multistep_params[stepId].step_index = stepId + 1;
        multistep.multistep_params[stepId].foot_index = stepId ^ 1;
        multistep.multistep_params[stepId].duration = this->strideDuration;

        angle = (stepId + 1) / 4.0 * this->turnAngle;

        double stepX = -sin(angle) * this->strideCoronal;
        double stepY = cos(angle) * this->strideCoronal;

        if ((stepId ^ 1) == 1)
        {
          stepX *= -1;
          stepY *= -1;
        }

        geometry_msgs::Pose pose;
        pose.position.x = stepX;
        pose.position.y = stepY;
        pose.position.z = 0;

        tf::Quaternion q(tf::Vector3(0, 0, 1), angle);
        tf::quaternionTFToMsg(q, pose.orientation);

        multistep.multistep_params[stepId].pose = pose;
      }
      goal.params.multistep_walk_params = multistep;

      dirty = true;
      break;
    }
    case KEYCODE_E:
    {
      std::cout << " turn right " << std::endl;

      double angle = 0;
      // for testing: generate 4 steps to match internal atlas step buffer size
      for ( int stepId = 0; stepId < 4; ++stepId)
      {
        multistep.multistep_params[stepId].step_index = stepId + 1;
        multistep.multistep_params[stepId].foot_index = stepId ^ 1;
        multistep.multistep_params[stepId].duration = this->strideDuration;

        angle = (stepId + 1) / 4.0 * this->turnAngle;

        double stepX = sin(angle) * this->strideCoronal;
        double stepY = -cos(angle) * this->strideCoronal;

        if ((stepId ^ 1) == 1)
        {
          stepX *= -1;
          stepY *= -1;
        }

        geometry_msgs::Pose pose;
        pose.position.x = stepX;
        pose.position.y = stepY;
        pose.position.z = 0;

        tf::Quaternion q(tf::Vector3(0, 0, 1), -angle);
        tf::quaternionTFToMsg(q, pose.orientation);

        multistep.multistep_params[stepId].pose = pose;
      }
      goal.params.multistep_walk_params = multistep;
      dirty = true;
      break;
    }
    }


    if (dirty == true)
    {
      client->sendGoal(goal,
        boost::bind(&TeleopAtlasKeyboard::resultCB, this, _1, _2),
        Client::SimpleActiveCallback(),
        boost::bind(&TeleopAtlasKeyboard::feedbackCB, this, _1));
      client->waitForResult(ros::Duration(10.0));
      if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        printf("The altas walk forward action suceeded");
    }


  }
}
