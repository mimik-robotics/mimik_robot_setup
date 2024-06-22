#include <mimik_ur3_cog/cog.h>
#include <mimik_ur3_cog/transform_frame.h>

#include <cmath>

using namespace mimik::ur3_cog;

geometry_msgs::InertiaStamped getFT300CouplingCOG();
geometry_msgs::InertiaStamped getFT300COG();
geometry_msgs::InertiaStamped getDualGripperAdapterCOG();
geometry_msgs::InertiaStamped getCameraAdapterCOG();
geometry_msgs::InertiaStamped getCameraCOG();
geometry_msgs::InertiaStamped get2F140COG();
geometry_msgs::InertiaStamped getPipetteCOG();
geometry_msgs::InertiaStamped getPipetteCableCOG();
geometry_msgs::InertiaStamped get2F140CableCOG();
geometry_msgs::InertiaStamped getFT300CableCOG();
geometry_msgs::InertiaStamped getCameraCableCOG();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur3_tool_cog");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<geometry_msgs::InertiaStamped> inertia_vec;

  inertia_vec.emplace_back(getFT300CouplingCOG());
  inertia_vec.emplace_back(getFT300COG());
  inertia_vec.emplace_back(getDualGripperAdapterCOG());
  inertia_vec.emplace_back(getCameraAdapterCOG());
  inertia_vec.emplace_back(getCameraCOG());
  inertia_vec.emplace_back(get2F140COG());
  inertia_vec.emplace_back(getPipetteCOG());
  inertia_vec.emplace_back(getPipetteCableCOG());
  inertia_vec.emplace_back(get2F140CableCOG());
  inertia_vec.emplace_back(getFT300CableCOG());
  inertia_vec.emplace_back(getCameraCableCOG());

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

  std::vector<COG> cog_vec{

    // Updated on 07/06/2024 from excel sheet
    COG{ .mass = 0.26486, .cog{ 0.01546, 0, -0.00005 } },          // Dual Gripper Adapter
    COG{ .mass = 0.003595, .cog{ 0.00333, -0.01768, 0.01768 } },   // Dual Gripper Adapter RS Screw 1
    COG{ .mass = 0.003595, .cog{ 0.00333, 0.01768, 0.01768 } },    // Dual Gripper Adapter RS Screw 2
    COG{ .mass = 0.003595, .cog{ 0.00333, 0.01768, -0.01768 } },   // Dual Gripper Adapter RS Screw 3
    COG{ .mass = 0.003595, .cog{ 0.00333, -0.01768, -0.01768 } },  // Dual Gripper Adapter RS Screw 4
    COG{ .mass = 0.00439, .cog{ 0.02983, 0.04583, 0.025 } },       // Dual Gripper Left Dowel Pin
    COG{ .mass = 0.0041, .cog{ 0.0058, 0.0468, 0.01768 } },        // Dual Gripper Left Screw (cable)
    COG{ .mass = 0.0015, .cog{ 0.0085, 0.0495, 0.01767 } },        // Dual Gripper Left Washer (cable)
    COG{ .mass = 0.00544, .cog{ 0.0085, 0.0495, 0.01767 } },       // Dual Gripper Left Clamp 1/4" (cable)
    COG{ .mass = 0.002, .cog{ 0.00795, -0.05931, 0 } },            // Dual Gripper Right Dowel Pin 1
    COG{ .mass = 0.002, .cog{ 0.02563, -0.04163, -0.025 } },       // Dual Gripper Right Dowel Pin 2
    COG{ .mass = 0.0097, .cog{ 0.0085, 0.0495, 0.01767 } },        // Dual Gripper Right Clamp 1/2"
    COG{ .mass = 0.00358, .cog{ 0.03483, -0.02583, -0.01768 } },   // Dual Gripper Right Screw 1 (on clamp)
    COG{ .mass = 0.00358, .cog{ 0.03624, -0.02724, 0.01768 } },    // Dual Gripper Right Screw 2
    COG{ .mass = 0.00358, .cog{ 0.01124, -0.05224, 0.01768 } },    // Dual Gripper Right Screw 3
    COG{ .mass = 0.00358, .cog{ 0.01124, -0.05224, -0.01768 } },   // Dual Gripper Right Screw 4

  };

  auto cog = computeCOG(cog_vec);

  COGToInertia(cog, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped getFT300CouplingCOG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "robotiq_ft300_coupling_robotside";

  std::vector<COG> cog_vec{

    // Updated on 07/06/2024 from excel sheet
    COG{ .mass = 0.05538, .cog{ 0.00226, -0.00157, -0.0004 } },      // FT300 Base
    COG{ .mass = 0.00038875, .cog{ 0.00183, -0.01768, 0.01768 } },   // FT300 Base Screw 1 w/ washer
    COG{ .mass = 0.00038875, .cog{ 0.00183, 0.01768, 0.01768 } },    // FT300 Base Screw 2 w/ washer
    COG{ .mass = 0.00038875, .cog{ 0.00183, 0.01768, -0.01768 } },   // FT300 Base Screw 3 w/ washer
    COG{ .mass = 0.00038875, .cog{ 0.00183, -0.01768, -0.01768 } },  // FT300 Base Screw 4 w/ washer
    COG{ .mass = 0.00257, .cog{ 0, 0, 0.025 } },                     // Dowel Pin (ur3_tool0 -> FTS-300)
  };

  auto cog = computeCOG(cog_vec);

  COGToInertia(cog, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped getFT300COG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "robotiq_ft300_robotside";

  std::vector<COG> cog_vec{

    // Updated on 07/06/2024 from excel sheet
    COG{ .mass = 0.2465, .cog{ 0.01962, 0, 0 } },                // FT300 w/ washers
    COG{ .mass = 0.00131, .cog{ 0.00392, 0.02788, 0.0142 } },    // FT300 Screw 1 (Above Torque S/N)
    COG{ .mass = 0.00131, .cog{ 0.00392, 0.01121, 0.02921 } },   // FT300 Screw 2 (Direction of Robotiq X axis)
    COG{ .mass = 0.00131, .cog{ 0.00392, -0.01121, 0.02921 } },  // FT300 Screw 3
    COG{ .mass = 0.00131, .cog{ 0.00392, -0.02788, 0.0142 } },   // FT300 Screw 4
    COG{ .mass = 0.00131, .cog{ 0.00392, -0.02788, -0.0142 } },  // FT300 Screw 5
    COG{ .mass = 0.00131, .cog{ 0.00392, 0, -0.03129 } },        // FT300 Screw 6
    COG{ .mass = 0.00131, .cog{ 0.00392, 0.02788, -0.0142 } },   // FT300 Screw 7
    COG{ .mass = 0.00257, .cog{ 0.03384, 0, 0.025 } },           // Dowel Pin (FT300-> Dual Gripper)
  };

  auto cog = computeCOG(cog_vec);

  COGToInertia(cog, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped get2F140COG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "robotiq_2f140_base_link";

  std::vector<COG> cog_vec{

    // From Robotiq 2019/03/29 instruction manual
    COG{ .mass = 1.025, .cog{ 0.073, 0, 0 } },  // Robotiq 2F140 Gripper Open

    // Updated on 07/06/2024 from excel sheet
    COG{ .mass = 0.00394, .cog{ 0.00121, 0.01768, 0.01768 } },   // 2F140 Base Screw 1 (Cut)
    COG{ .mass = 0.005, .cog{ -0.00102, 0.01768, -0.01768 } },   // 2F140 Base Screw 2
    COG{ .mass = 0.005, .cog{ -0.00102, -0.01768, -0.01768 } },  // 2F140 Base Screw 3
    COG{ .mass = 0.005, .cog{ -0.00102, -0.01768, 0.01768 } },   // 2F140 Base Screw 4

  };

  auto cog = computeCOG(cog_vec);

  COGToInertia(cog, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped getCameraAdapterCOG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "camera_adapter_robotside";

  std::vector<COG> cog_vec{

    // Updated on 07/06/2024 from excel sheet
    COG{ .mass = 0.02652, .cog{ 0.00261, 0, 0.02117 } },          // Camera Adapter
    COG{ .mass = 0.00047, .cog{ 0.00582, -0.022499, 0.08016 } },  // Camera Screw 1
    COG{ .mass = 0.00047, .cog{ 0.00582, 0.022499, 0.08016 } },   // Camera Screw 2

  };

  auto cog = computeCOG(cog_vec);

  COGToInertia(cog, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped getCameraCOG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "camera_adapter_camera";

  // approximate to centroid of camera
  inertia.inertia.com.x = 0.020 / 2.0;
  inertia.inertia.com.y = 0;
  inertia.inertia.com.z = 0;

  // // without usb cap
  inertia.inertia.m = 0.06005;

  return inertia;
}

geometry_msgs::InertiaStamped getPipetteCableCOG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "pipette_base";

  std::vector<COG> cog_vec{

    // Updated on 17/06/2024 from excel sheet
    COG{ .mass = 0.00529, .cog{ 0.00496, 0.04226, -0.00993 } }
  };

  auto cog = computeCOG(cog_vec);

  COGToInertia(cog, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped get2F140CableCOG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "robotiq_2f140_base_link";

  std::vector<COG> cog_vec{

    // Updated on 17/06/2024 from excel sheet
    COG{ .mass = 0.0062, .cog{ -0.0051, 0.00201, -0.02863 } }
  };

  auto cog = computeCOG(cog_vec);

  COGToInertia(cog, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped getFT300CableCOG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "robotiq_ft300_robotside";

  std::vector<COG> cog_vec{

    // Updated on 17/06/2024 from excel sheet
    COG{ .mass = 0.00869, .cog{ 0.03974, 0.05923, -0.02493 } }
  };

  auto cog = computeCOG(cog_vec);

  COGToInertia(cog, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped getCameraCableCOG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "camera_adapter_robotside";

  std::vector<COG> cog_vec{

    // Updated on 17/06/2024 from excel sheet
    COG{ .mass = 0.02532, .cog{ 0.00381, -0.03602, 0.01735 } }
  };

  auto cog = computeCOG(cog_vec);

  COGToInertia(cog, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped getPipetteCOG()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "pipette_base";

  std::vector<COG> cog_vec{

    // Updated on 07/06/2024 from excel sheet
    COG{ .mass = 0.2003557, .cog{ 0.05776, -0.00006, -0.00063 } },  // Pipette Frame
    COG{ .mass = 0.128, .cog{ 0.03315, 0, -0.01261 } },             // Nema 17
    COG{ .mass = 0.0157, .cog{ 0.0165, 0, -0.0125 } },              // Encoder
    COG{ .mass = 0.0079278, .cog{ 0.05741, 0, -0.0125 } },          // Clamp Flexible Shaft
    COG{ .mass = 0.00522, .cog{ 0.09788, 0, -0.0125 } },            // Lead Screw
    COG{ .mass = 0.00772, .cog{ 0.08874, 0, 0.00909 } },            // Guide Rail
    COG{ .mass = 0.00427, .cog{ 0.1181275, 0.01582, -0.0125 } },    // Tip Enclosure
    COG{ .mass = 0.00077, .cog{ 0.12773, 0.01294, -0.0125 } },      // Tip Enclosure Slot
    COG{ .mass = 0.011, .cog{ 0.1454, 0, 0 } },                     // Piston assembly (Gilson P200)
    COG{ .mass = 0.01, .cog{ 0.18169, 0, 0 } },                     // Pipette Tip Holder + Nut
    COG{ .mass = 0.0077, .cog{ 0.18519, 0.00491, -0.00501 } },      // Pipette Tip Ejector
    COG{ .mass = 0.00354, .cog{ 0.11198, 0.0125, -0.0125 } },       // Pipette Ejector Piston
    COG{ .mass = 0.006, .cog{ 0.06997, -0.00003, -0.01197 } },      // Bearing Support
    COG{ .mass = 0.00387, .cog{ 0.10036, 0.0096, -0.00634 } },      // Flange Nut Housing + Magnet Cover
    COG{ .mass = 0.00548, .cog{ 0.08944, 0.02911, 0.02359 } },      // PCB Guide Left
    COG{ .mass = 0.00548, .cog{ 0.08944, -0.02911, 0.02359 } },     // PCB Guide Right
    COG{ .mass = 0.00147, .cog{ 0.09801, 0, -0.0125 } },            // Fast Travel Flange
    COG{ .mass = 0.00678, .cog{ 0.09, -0.00004, 0.00711 } },        // Ball Bearing Carriage
    COG{ .mass = 0.00075, .cog{ 0.02353, 0.01549, -0.02799 } },     // Motor LockNut 1
    COG{ .mass = 0.00075, .cog{ 0.02353, 0.01549, 0.00299 } },      // Motor LockNut 2
    COG{ .mass = 0.00075, .cog{ 0.02353, -0.01549, 0.00299 } },     // Motor LockNut 3
    COG{ .mass = 0.00075, .cog{ 0.02353, -0.01549, -0.02799 } },    // Motor LockNut 4
    COG{ .mass = 0.00148, .cog{ 0.03776, 0.01549, -0.02799 } },     // Motor Screw 1
    COG{ .mass = 0.00148, .cog{ 0.03776, 0.01549, 0.00299 } },      // Motor Screw 2
    COG{ .mass = 0.00148, .cog{ 0.03776, -0.01549, 0.00299 } },     // Motor Screw 3
    COG{ .mass = 0.00148, .cog{ 0.03776, -0.01549, -0.02799 } },    // Motor Screw 4
    COG{ .mass = 0.000739, .cog{ 0.06949, 0.02884, -0.01752 } },    // Bearing Support Screw 1
    COG{ .mass = 0.000739, .cog{ 0.06949, 0.02884, -0.00752 } },    // Bearing Support Screw 2
    COG{ .mass = 0.000739, .cog{ 0.06949, -0.02884, -0.00752 } },   // Bearing Support Screw 3
    COG{ .mass = 0.000739, .cog{ 0.06949, -0.02884, -0.01752 } },   // Bearing Support Screw 4
    COG{ .mass = 0.00037, .cog{ 0.07161, 0, -0.01252 } },           // Ball Bearing (in support)
    COG{ .mass = 0.0018173, .cog{ 0.07524, 0, -0.01252 } },         // Thrust Ball Bearing 1
    COG{ .mass = 0.0018173, .cog{ 0.12524, 0, -0.0125 } },          // Thrust Ball Bearing 2
    COG{ .mass = 0.00258, .cog{ 0.07974, 0, -0.0125 } },            // Shaft Collar 1 + Set Screw
    COG{ .mass = 0.00258, .cog{ 0.12074, 0, -0.01249 } },           // Shaft Collar 2 + Set Screw
    COG{ .mass = 0.00053, .cog{ 0.09612, 0, -0.02012 } },           // Fast Travel Flange Screw 1
    COG{ .mass = 0.00053, .cog{ 0.09612, 0, -0.00488 } },           // Fast Travel Flange Screw 2
    COG{ .mass = 0.0002721, .cog{ 0.09, 0.004, 0.00084 } },         // Magnet Cover Screw 1
    COG{ .mass = 0.0002721, .cog{ 0.09, -0.004, 0.00084 } },        // Magnet Cover Screw 2
    COG{ .mass = 0.00067, .cog{ 0.12713, 0.01528, -0.02398 } },     // Tip Ejector Enclosure Screw 1
    COG{ .mass = 0.00067, .cog{ 0.12713, 0.01528, -0.00102 } },     // Tip Ejector Enclosure Screw 2
    COG{ .mass = 0.00037, .cog{ 0.13075, 0, -0.0125 } },            // Ball Bearing (end of lead screw)
    COG{ .mass = 0.00031, .cog{ 0.05076, 0.028, 0.028 } },          // PCB Guide Left Screw 1
    COG{ .mass = 0.0003, .cog{ 0.06319, 0.0295, 0.02106 } },        // PCB Guide Left Screw 2
    COG{ .mass = 0.0003, .cog{ 0.12019, 0.0295, 0.02106 } },        // PCB Guide Left Screw 3
    COG{ .mass = 0.00031, .cog{ 0.05076, -0.028, 0.028 } },         // PCB Guide Rigth Screw 1
    COG{ .mass = 0.0003, .cog{ 0.06319, -0.0295, 0.02106 } },       // PCB Guide Rigth Screw 2
    COG{ .mass = 0.0003, .cog{ 0.12019, -0.0295, 0.02106 } },       // PCB Guide Rigth Screw 3
    COG{ .mass = 0.00031, .cog{ 0.05819, 0.021, 0.01793 } },        // PCB Screw
    COG{ .mass = 0.00221, .cog{ -0.001, 0, -0.025 } },              // Dowel Pin 1
    COG{ .mass = 0.00221, .cog{ -0.001, -0.025, 0 } },              // Dowel Pin 2
    COG{ .mass = 0.00025, .cog{ 0.11833, 0.01258, -0.01248 } },     // Compression Spring
    COG{ .mass = 0.00997, .cog{ 0.08906, 0.00008, 0.01876 } },      // Plain PCB
    COG{ .mass = 0.0055, .cog{ 0.079585, -0.0131, 0.03035 } },      // PCB RS-485
    COG{ .mass = 0.003, .cog{ 0.11949, -0.01992, 0.03035 } },       // PCB Driver Motor + Female header
    COG{ .mass = 0.0097, .cog{ 0.091, 0.01346, 0.03035 } },         // PCB Arduino uC + Female header
    COG{ .mass = 0.0187317, .cog{ 0.08906, 0, 0.01876 } },          // PCB (Remaining)

  };

  auto cog = computeCOG(cog_vec);

  COGToInertia(cog, inertia.inertia);

  return inertia;
}
