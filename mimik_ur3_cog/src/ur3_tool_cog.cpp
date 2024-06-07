#include <mimik_ur3_cog/cog.h>
#include <mimik_ur3_cog/transform_pose.h>

#include <cmath>

using namespace mimik::ur3_cog;

geometry_msgs::InertiaStamped getDualGripperAdapterCOG();
geometry_msgs::InertiaStamped getFT300COG();
geometry_msgs::InertiaStamped get2F140COG();
geometry_msgs::InertiaStamped getCameraAdapterCOG();
geometry_msgs::InertiaStamped getCameraCOG();
geometry_msgs::InertiaStamped getCameraUSBCOG();
geometry_msgs::InertiaStamped getPipetteCOG();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur3_tool_cog");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<geometry_msgs::InertiaStamped> inertia_vec;

  inertia_vec.emplace_back(getFT300COG());
  inertia_vec.emplace_back(getDualGripperAdapterCOG());
  inertia_vec.emplace_back(getCameraAdapterCOG());
  inertia_vec.emplace_back(getCameraCOG());
  inertia_vec.emplace_back(get2F140COG());
  inertia_vec.emplace_back(getPipetteCOG());

  std::string to_frame = "ur3_tool0";
  auto cog = computeCOG(inertia_vec, to_frame);

  // print CoG
  std::cout << "Computing CoG wrt. " << to_frame << std::endl;
  std::cout << "===================";
  for (std::size_t i = 0; i < to_frame.size(); ++i)
    std::cout << "=";
  std::cout << std::endl;

  print(cog);

  // print CoG Distance
  double distance = std::hypot(cog.cog.x(), cog.cog.y(), cog.cog.z());
  std::cout << "\ndistance from " << to_frame << " = " << distance << std::endl;
}

geometry_msgs::InertiaStamped getDualGripperAdapterCOG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "dual_gripper_coupling_robotside";

  inertia.inertia.com.x = 0.01545;
  inertia.inertia.com.y = 0;
  inertia.inertia.com.z = 0;
  inertia.inertia.m = 0.283;

  return inertia;
}

geometry_msgs::InertiaStamped getFT300COG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "robotiq_ft300_coupling_robotside";

  inertia.inertia.com.x = 0.017;
  inertia.inertia.com.y = 0;
  inertia.inertia.com.z = 0;
  inertia.inertia.m = 0.300;

  return inertia;
}

geometry_msgs::InertiaStamped get2F140COG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "robotiq_2f140_base_link";

  // From Robotiq 2019/03/29 instruction manual
  inertia.inertia.com.x = 0.073;
  inertia.inertia.com.y = 0;
  inertia.inertia.com.z = 0;
  inertia.inertia.m = 1.025;

  return inertia;
}

geometry_msgs::InertiaStamped getCameraAdapterCOG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "camera_adapter_robotside";

  // approximate to center of camera
  inertia.inertia.com.x = 0.002614;
  inertia.inertia.com.y = 0;
  inertia.inertia.com.z = 0.02117;
  inertia.inertia.m = 0.02953;

  return inertia;
}

geometry_msgs::InertiaStamped getCameraCOG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "camera_adapter_camera";

  // approximate to center of camera
  inertia.inertia.com.x = 0.020 / 2.0;
  inertia.inertia.com.y = 0;
  inertia.inertia.com.z = 0;

  // without usb cap
  inertia.inertia.m = 0.064 - 0.0023;

  return inertia;
}

geometry_msgs::InertiaStamped getCameraUSBCOG()
{
}

geometry_msgs::InertiaStamped getPipetteCOG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "pipette_base";

  std::vector<COG> cog_vec{

    // Updated on 04/06/2024 from excel sheet
    COG{ .mass = 0.2, .cog{ 0.05776, -0.00006, -0.00063 } },       // Frame
    COG{ .mass = 0.128, .cog{ 0.03315, 0, -0.01261 } },            // Nema 17
    COG{ .mass = 0.0157, .cog{ 0.0165, 0, -0.0125 } },             // Encoder
    COG{ .mass = 0.0079278, .cog{ 0.05741, 0, -0.0125 } },         // Clamp Flexible Shaft
    COG{ .mass = 0.00522, .cog{ 0.09788, 0, -0.0125 } },           // Lead Screw
    COG{ .mass = 0.00772, .cog{ 0.08874, 0, 0.00909 } },           // Guide Rail
    COG{ .mass = 0.00427, .cog{ 0.1181275, 0.01582, -0.0125 } },   // Tip Enclosure
    COG{ .mass = 0.00077, .cog{ 0.12773, 0.01294, -0.0125 } },     // Tip Enclosure Slot
    COG{ .mass = 0.011, .cog{ 0.1454, 0, 0 } },                    // Piston assembly (Gilson P200)
    COG{ .mass = 0.01, .cog{ 0.18169, 0, 0 } },                    // Pipette Tip Holder + Nut
    COG{ .mass = 0.0077, .cog{ 0.18519, 0.00491, -0.00501 } },     // Pipette Tip Ejector
    COG{ .mass = 0.00997, .cog{ 0.08906, 0.00008, 0.01876 } },     // Plain PCB
    COG{ .mass = 0.00354, .cog{ 0.11198, 0.0125, -0.0125 } },      // Pipette Ejector Piston
    COG{ .mass = 0.006, .cog{ 0.06997, -0.00003, -0.01197 } },     // Bearing Support
    COG{ .mass = 0.00387, .cog{ 0.10036, 0.0096, -0.00634 } },     // Flange Nut Housing + Magnet Cover
    COG{ .mass = 0.00548, .cog{ 0.08944, 0.02911, 0.02359 } },     // PCB Guide Left
    COG{ .mass = 0.00548, .cog{ 0.08944, -0.02911, 0.02359 } },    // PCB Guide Right
    COG{ .mass = 0.00147, .cog{ 0.09801, 0, -0.0125 } },           // Fast Travel Flange
    COG{ .mass = 0.00678, .cog{ 0.09, -0.00004, 0.00711 } },       // Ball Bearing Carriage
    COG{ .mass = 0.00075, .cog{ 0.02353, 0.01549, -0.02799 } },    // Motor LockNut 1
    COG{ .mass = 0.00075, .cog{ 0.02353, 0.01549, 0.00299 } },     // Motor LockNut 2
    COG{ .mass = 0.00075, .cog{ 0.02353, -0.01549, 0.00299 } },    // Motor LockNut 3
    COG{ .mass = 0.00075, .cog{ 0.02353, -0.01549, -0.02799 } },   // Motor LockNut 4
    COG{ .mass = 0.00148, .cog{ 0.03776, 0.01549, -0.02799 } },    // Motor Screw 1
    COG{ .mass = 0.00148, .cog{ 0.03776, 0.01549, 0.00299 } },     // Motor Screw 2
    COG{ .mass = 0.00148, .cog{ 0.03776, -0.01549, 0.00299 } },    // Motor Screw 3
    COG{ .mass = 0.00148, .cog{ 0.03776, -0.01549, -0.02799 } },   // Motor Screw 4
    COG{ .mass = 0.000739, .cog{ 0.06949, 0.02884, -0.01752 } },   // Bearing Support Screw 1
    COG{ .mass = 0.000739, .cog{ 0.06949, 0.02884, -0.00752 } },   // Bearing Support Screw 2
    COG{ .mass = 0.000739, .cog{ 0.06949, -0.02884, -0.00752 } },  // Bearing Support Screw 3
    COG{ .mass = 0.000739, .cog{ 0.06949, -0.02884, -0.01752 } },  // Bearing Support Screw 4
    COG{ .mass = 0.00037, .cog{ 0.07161, 0, -0.01252 } },          // Ball Bearing (in support)
    COG{ .mass = 0.0018173, .cog{ 0.07524, 0, -0.01252 } },        // Thrust Ball Bearing 1
    COG{ .mass = 0.0018173, .cog{ 0.12524, 0, -0.0125 } },         // Thrust Ball Bearing 2
    COG{ .mass = 0.00258, .cog{ 0.07974, 0, -0.0125 } },           // Shaft Collar 1 + Set Screw
    COG{ .mass = 0.00258, .cog{ 0.12074, 0, -0.01249 } },          // Shaft Collar 2 + Set Screw
    COG{ .mass = 0.00053, .cog{ 0.09612, 0, -0.02012 } },          // Fast Travel Flange Screw 1
    COG{ .mass = 0.00053, .cog{ 0.09612, 0, -0.00488 } },          // Fast Travel Flange Screw 2
    COG{ .mass = 0.0002721, .cog{ 0.09, 0.004, 0.00084 } },        // Magnet Cover Screw 1
    COG{ .mass = 0.0002721, .cog{ 0.09, -0.004, 0.00084 } },       // Magnet Cover Screw 2
    COG{ .mass = 0.00067, .cog{ 0.12713, 0.01528, -0.02398 } },    // Tip Ejector Enclosure Screw 1
    COG{ .mass = 0.00067, .cog{ 0.12713, 0.01528, -0.00102 } },    // Tip Ejector Enclosure Screw 2
    COG{ .mass = 0.00037, .cog{ 0.13075, 0, -0.0125 } },           // Ball Bearing (end of lead screw)
    COG{ .mass = 0.00031, .cog{ 0.05076, 0.028, 0.028 } },         // PCB Guide Left Screw 1
    COG{ .mass = 0.0003, .cog{ 0.06319, 0.0295, 0.02106 } },       // PCB Guide Left Screw 2
    COG{ .mass = 0.0003, .cog{ 0.12019, 0.0295, 0.02106 } },       // PCB Guide Left Screw 3
    COG{ .mass = 0.00031, .cog{ 0.05076, -0.028, 0.028 } },        // PCB Guide Rigth Screw 1
    COG{ .mass = 0.0003, .cog{ 0.06319, -0.0295, 0.02106 } },      // PCB Guide Rigth Screw 2
    COG{ .mass = 0.0003, .cog{ 0.12019, -0.0295, 0.02106 } },      // PCB Guide Rigth Screw 3
    COG{ .mass = 0.00031, .cog{ 0.05819, 0.021, 0.01793 } },       // PCB Screw
    COG{ .mass = 0, .cog{ 0, 0, 0 } },                             // Top Screw 1
    COG{ .mass = 0, .cog{ 0, 0, 0 } },                             // Top Screw 2
    COG{ .mass = 0, .cog{ 0, 0, 0 } },                             // Top Screw 3
    COG{ .mass = 0, .cog{ 0, 0, 0 } },                             // Top Screw 4
    COG{ .mass = 0.00221, .cog{ -0.001, 0, -0.025 } },             // Dowel Pin 1
    COG{ .mass = 0.00221, .cog{ -0.001, -0.025, 0 } },             // Dowel Pin 2
    COG{ .mass = 0.00025, .cog{ 0.11833, 0.01258, -0.01248 } },    // Compression Spring
  };

  auto cog = computeCOG(cog_vec);

  COGToInertia(cog, inertia.inertia);

  return inertia;
}
