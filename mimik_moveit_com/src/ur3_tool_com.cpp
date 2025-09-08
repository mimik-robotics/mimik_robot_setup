#include <mimik_moveit_com/com.h>
#include <mimik_moveit_com/transform_frame.h>

#include <cmath>

using namespace mimik_moveit_com;

geometry_msgs::InertiaStamped getFT300CouplingCoM();
geometry_msgs::InertiaStamped getFT300CoM();
geometry_msgs::InertiaStamped getDualGripperAdapterCoM();
geometry_msgs::InertiaStamped getMimikCameraAdapterCoM();
geometry_msgs::InertiaStamped getPicknikCameraAdapterCoM();
geometry_msgs::InertiaStamped getCameraCoM();
geometry_msgs::InertiaStamped get2F140CoM();
geometry_msgs::InertiaStamped getPipetteCoM();
geometry_msgs::InertiaStamped getPipetteCableCoM();
geometry_msgs::InertiaStamped get2F140CableCoM();
geometry_msgs::InertiaStamped getFT300CableCoM();
geometry_msgs::InertiaStamped getCameraCableCoM();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur3_tool_com");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<geometry_msgs::InertiaStamped> inertia_vec;

  inertia_vec.emplace_back(getFT300CouplingCoM());
  inertia_vec.emplace_back(getFT300CoM());
  inertia_vec.emplace_back(getDualGripperAdapterCoM());
  inertia_vec.emplace_back(getMimikCameraAdapterCoM());
  inertia_vec.emplace_back(getCameraCoM());
  inertia_vec.emplace_back(get2F140CoM());
  inertia_vec.emplace_back(getPipetteCoM());
  inertia_vec.emplace_back(getPipetteCableCoM());
  inertia_vec.emplace_back(get2F140CableCoM());
  inertia_vec.emplace_back(getFT300CableCoM());
  inertia_vec.emplace_back(getCameraCableCoM());

  std::string to_frame = "ur3_tool0";
  auto com = computeCoM(inertia_vec, to_frame);

  // print Com
  std::cout << "Computing Com wrt. " << to_frame << std::endl;
  std::cout << "===================";
  for (std::size_t i = 0; i < to_frame.size(); ++i)
    std::cout << "=";
  std::cout << std::endl;

  print(com);

  // print Com Distance
  double distance = std::hypot(com.com.x(), com.com.y(), com.com.z());
  std::cout << "\ndistance from " << to_frame << " = " << distance << std::endl;
}

geometry_msgs::InertiaStamped getDualGripperAdapterCoM()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "dual_gripper_coupling_robotside";

  std::vector<CoM> com_vec{

    // Updated on 07/06/2024 from excel sheet
    CoM{ .mass = 0.26486, .com{ 0.01546, 0, -0.00005 } },          // Dual Gripper Adapter
    CoM{ .mass = 0.003595, .com{ 0.00333, -0.01768, 0.01768 } },   // Dual Gripper Adapter RS Screw 1
    CoM{ .mass = 0.003595, .com{ 0.00333, 0.01768, 0.01768 } },    // Dual Gripper Adapter RS Screw 2
    CoM{ .mass = 0.003595, .com{ 0.00333, 0.01768, -0.01768 } },   // Dual Gripper Adapter RS Screw 3
    CoM{ .mass = 0.003595, .com{ 0.00333, -0.01768, -0.01768 } },  // Dual Gripper Adapter RS Screw 4
    CoM{ .mass = 0.00439, .com{ 0.02983, 0.04583, 0.025 } },       // Dual Gripper Left Dowel Pin
    CoM{ .mass = 0.0041, .com{ 0.0058, 0.0468, 0.01768 } },        // Dual Gripper Left Screw (cable)
    CoM{ .mass = 0.0015, .com{ 0.0085, 0.0495, 0.01767 } },        // Dual Gripper Left Washer (cable)
    CoM{ .mass = 0.00544, .com{ 0.0085, 0.0495, 0.01767 } },       // Dual Gripper Left Clamp 1/4" (cable)
    CoM{ .mass = 0.002, .com{ 0.00795, -0.05931, 0 } },            // Dual Gripper Right Dowel Pin 1
    CoM{ .mass = 0.002, .com{ 0.02563, -0.04163, -0.025 } },       // Dual Gripper Right Dowel Pin 2
    CoM{ .mass = 0.0097, .com{ 0.0085, 0.0495, 0.01767 } },        // Dual Gripper Right Clamp 1/2"
    CoM{ .mass = 0.00358, .com{ 0.03483, -0.02583, -0.01768 } },   // Dual Gripper Right Screw 1 (on clamp)
    CoM{ .mass = 0.00358, .com{ 0.03624, -0.02724, 0.01768 } },    // Dual Gripper Right Screw 2
    CoM{ .mass = 0.00358, .com{ 0.01124, -0.05224, 0.01768 } },    // Dual Gripper Right Screw 3
    CoM{ .mass = 0.00358, .com{ 0.01124, -0.05224, -0.01768 } },   // Dual Gripper Right Screw 4

  };

  auto com = computeCoM(com_vec);

  CoMToInertia(com, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped getFT300CouplingCoM()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "robotiq_ft300_coupling_robotside";

  std::vector<CoM> com_vec{

    // Updated on 07/06/2024 from excel sheet
    CoM{ .mass = 0.05538, .com{ 0.00226, -0.00157, -0.0004 } },      // FT300 Base
    CoM{ .mass = 0.00038875, .com{ 0.00183, -0.01768, 0.01768 } },   // FT300 Base Screw 1 w/ washer
    CoM{ .mass = 0.00038875, .com{ 0.00183, 0.01768, 0.01768 } },    // FT300 Base Screw 2 w/ washer
    CoM{ .mass = 0.00038875, .com{ 0.00183, 0.01768, -0.01768 } },   // FT300 Base Screw 3 w/ washer
    CoM{ .mass = 0.00038875, .com{ 0.00183, -0.01768, -0.01768 } },  // FT300 Base Screw 4 w/ washer
    CoM{ .mass = 0.00257, .com{ 0, 0, 0.025 } },                     // Dowel Pin (ur3_tool0 -> FTS-300)
  };

  auto com = computeCoM(com_vec);

  CoMToInertia(com, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped getFT300CoM()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "robotiq_ft300_robotside";

  std::vector<CoM> com_vec{

    // Updated on 07/06/2024 from excel sheet
    CoM{ .mass = 0.2465, .com{ 0.01962, 0, 0 } },                // FT300 w/ washers
    CoM{ .mass = 0.00131, .com{ 0.00392, 0.02788, 0.0142 } },    // FT300 Screw 1 (Above Torque S/N)
    CoM{ .mass = 0.00131, .com{ 0.00392, 0.01121, 0.02921 } },   // FT300 Screw 2 (Direction of Robotiq X axis)
    CoM{ .mass = 0.00131, .com{ 0.00392, -0.01121, 0.02921 } },  // FT300 Screw 3
    CoM{ .mass = 0.00131, .com{ 0.00392, -0.02788, 0.0142 } },   // FT300 Screw 4
    CoM{ .mass = 0.00131, .com{ 0.00392, -0.02788, -0.0142 } },  // FT300 Screw 5
    CoM{ .mass = 0.00131, .com{ 0.00392, 0, -0.03129 } },        // FT300 Screw 6
    CoM{ .mass = 0.00131, .com{ 0.00392, 0.02788, -0.0142 } },   // FT300 Screw 7
    CoM{ .mass = 0.00257, .com{ 0.03384, 0, 0.025 } },           // Dowel Pin (FT300-> Dual Gripper)
  };

  auto com = computeCoM(com_vec);

  CoMToInertia(com, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped get2F140CoM()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "robotiq_2f140_base_link";

  std::vector<CoM> com_vec{

    // From Robotiq 2019/03/29 instruction manual
    CoM{ .mass = 1.025, .com{ 0.073, 0, 0 } },  // Robotiq 2F140 Gripper Open

    // Updated on 07/06/2024 from excel sheet
    CoM{ .mass = 0.00394, .com{ 0.00121, 0.01768, 0.01768 } },   // 2F140 Base Screw 1 (Cut)
    CoM{ .mass = 0.005, .com{ -0.00102, 0.01768, -0.01768 } },   // 2F140 Base Screw 2
    CoM{ .mass = 0.005, .com{ -0.00102, -0.01768, -0.01768 } },  // 2F140 Base Screw 3
    CoM{ .mass = 0.005, .com{ -0.00102, -0.01768, 0.01768 } },   // 2F140 Base Screw 4

  };

  auto com = computeCoM(com_vec);

  CoMToInertia(com, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped getMimikCameraAdapterCoM()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "camera_adapter_robotside";

  std::vector<CoM> com_vec{

    // Updated on 07/06/2024 from excel sheet
    CoM{ .mass = 0.05709, .com{ 0.017471, 0.0, 0.006842 } },  // Camera Adapter
    CoM{ .mass = 0.00047, .com{ 0.042, 0.0225, 0.016 } },     // Camera Screw 1
    CoM{ .mass = 0.00047, .com{ 0.042, -0.0225, 0.016 } },    // Camera Screw 2

  };

  auto com = computeCoM(com_vec);

  CoMToInertia(com, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped getPicknikCameraAdapterCoM()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "camera_adapter_robotside";

  std::vector<CoM> com_vec{

    // Updated on 07/06/2024 from excel sheet
    CoM{ .mass = 0.02652, .com{ 0.00261, 0, 0.02117 } },          // Camera Adapter
    CoM{ .mass = 0.00047, .com{ 0.00582, -0.022499, 0.08016 } },  // Camera Screw 1
    CoM{ .mass = 0.00047, .com{ 0.00582, 0.022499, 0.08016 } },   // Camera Screw 2

  };

  auto com = computeCoM(com_vec);

  CoMToInertia(com, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped getCameraCoM()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "camera_adapter_toolside";

  // approximate to centroid of camera
  inertia.inertia.com.x = 0.020 / 2.0;
  inertia.inertia.com.y = 0;
  inertia.inertia.com.z = 0;

  // // without usb cap
  inertia.inertia.m = 0.06005;

  return inertia;
}

geometry_msgs::InertiaStamped getPipetteCableCoM()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "pipette_base";

  std::vector<CoM> com_vec{

    // Updated on 17/06/2024 from excel sheet
    CoM{ .mass = 0.00529, .com{ 0.00496, 0.04226, -0.00993 } }
  };

  auto com = computeCoM(com_vec);

  CoMToInertia(com, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped get2F140CableCoM()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "robotiq_2f140_base_link";

  std::vector<CoM> com_vec{

    // Updated on 17/06/2024 from excel sheet
    CoM{ .mass = 0.0062, .com{ -0.0051, 0.00201, -0.02863 } }
  };

  auto com = computeCoM(com_vec);

  CoMToInertia(com, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped getFT300CableCoM()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "robotiq_ft300_robotside";

  std::vector<CoM> com_vec{

    // Updated on 17/06/2024 from excel sheet
    CoM{ .mass = 0.00869, .com{ 0.03974, 0.05923, -0.02493 } }
  };

  auto com = computeCoM(com_vec);

  CoMToInertia(com, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped getCameraCableCoM()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "camera_adapter_robotside";

  std::vector<CoM> com_vec{

    // Updated on 17/06/2024 from excel sheet
    CoM{ .mass = 0.02532, .com{ 0.00381, -0.03602, 0.01735 } }
  };

  auto com = computeCoM(com_vec);

  CoMToInertia(com, inertia.inertia);

  return inertia;
}

geometry_msgs::InertiaStamped getPipetteCoM()
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = "pipette_base";

  std::vector<CoM> com_vec{

    // Updated on 07/06/2024 from excel sheet
    CoM{ .mass = 0.2003557, .com{ 0.05776, -0.00006, -0.00063 } },  // Pipette Frame
    CoM{ .mass = 0.128, .com{ 0.03315, 0, -0.01261 } },             // Nema 17
    CoM{ .mass = 0.0157, .com{ 0.0165, 0, -0.0125 } },              // Encoder
    CoM{ .mass = 0.0079278, .com{ 0.05741, 0, -0.0125 } },          // Clamp Flexible Shaft
    CoM{ .mass = 0.00522, .com{ 0.09788, 0, -0.0125 } },            // Lead Screw
    CoM{ .mass = 0.00772, .com{ 0.08874, 0, 0.00909 } },            // Guide Rail
    CoM{ .mass = 0.00427, .com{ 0.1181275, 0.01582, -0.0125 } },    // Tip Enclosure
    CoM{ .mass = 0.00077, .com{ 0.12773, 0.01294, -0.0125 } },      // Tip Enclosure Slot
    CoM{ .mass = 0.011, .com{ 0.1454, 0, 0 } },                     // Piston assembly (Gilson P200)
    CoM{ .mass = 0.01, .com{ 0.18169, 0, 0 } },                     // Pipette Tip Holder + Nut
    CoM{ .mass = 0.0077, .com{ 0.18519, 0.00491, -0.00501 } },      // Pipette Tip Ejector
    CoM{ .mass = 0.00354, .com{ 0.11198, 0.0125, -0.0125 } },       // Pipette Ejector Piston
    CoM{ .mass = 0.006, .com{ 0.06997, -0.00003, -0.01197 } },      // Bearing Support
    CoM{ .mass = 0.00387, .com{ 0.10036, 0.0096, -0.00634 } },      // Flange Nut Housing + Magnet Cover
    CoM{ .mass = 0.00548, .com{ 0.08944, 0.02911, 0.02359 } },      // PCB Guide Left
    CoM{ .mass = 0.00548, .com{ 0.08944, -0.02911, 0.02359 } },     // PCB Guide Right
    CoM{ .mass = 0.00147, .com{ 0.09801, 0, -0.0125 } },            // Fast Travel Flange
    CoM{ .mass = 0.00678, .com{ 0.09, -0.00004, 0.00711 } },        // Ball Bearing Carriage
    CoM{ .mass = 0.00075, .com{ 0.02353, 0.01549, -0.02799 } },     // Motor LockNut 1
    CoM{ .mass = 0.00075, .com{ 0.02353, 0.01549, 0.00299 } },      // Motor LockNut 2
    CoM{ .mass = 0.00075, .com{ 0.02353, -0.01549, 0.00299 } },     // Motor LockNut 3
    CoM{ .mass = 0.00075, .com{ 0.02353, -0.01549, -0.02799 } },    // Motor LockNut 4
    CoM{ .mass = 0.00148, .com{ 0.03776, 0.01549, -0.02799 } },     // Motor Screw 1
    CoM{ .mass = 0.00148, .com{ 0.03776, 0.01549, 0.00299 } },      // Motor Screw 2
    CoM{ .mass = 0.00148, .com{ 0.03776, -0.01549, 0.00299 } },     // Motor Screw 3
    CoM{ .mass = 0.00148, .com{ 0.03776, -0.01549, -0.02799 } },    // Motor Screw 4
    CoM{ .mass = 0.000739, .com{ 0.06949, 0.02884, -0.01752 } },    // Bearing Support Screw 1
    CoM{ .mass = 0.000739, .com{ 0.06949, 0.02884, -0.00752 } },    // Bearing Support Screw 2
    CoM{ .mass = 0.000739, .com{ 0.06949, -0.02884, -0.00752 } },   // Bearing Support Screw 3
    CoM{ .mass = 0.000739, .com{ 0.06949, -0.02884, -0.01752 } },   // Bearing Support Screw 4
    CoM{ .mass = 0.00037, .com{ 0.07161, 0, -0.01252 } },           // Ball Bearing (in support)
    CoM{ .mass = 0.0018173, .com{ 0.07524, 0, -0.01252 } },         // Thrust Ball Bearing 1
    CoM{ .mass = 0.0018173, .com{ 0.12524, 0, -0.0125 } },          // Thrust Ball Bearing 2
    CoM{ .mass = 0.00258, .com{ 0.07974, 0, -0.0125 } },            // Shaft Collar 1 + Set Screw
    CoM{ .mass = 0.00258, .com{ 0.12074, 0, -0.01249 } },           // Shaft Collar 2 + Set Screw
    CoM{ .mass = 0.00053, .com{ 0.09612, 0, -0.02012 } },           // Fast Travel Flange Screw 1
    CoM{ .mass = 0.00053, .com{ 0.09612, 0, -0.00488 } },           // Fast Travel Flange Screw 2
    CoM{ .mass = 0.0002721, .com{ 0.09, 0.004, 0.00084 } },         // Magnet Cover Screw 1
    CoM{ .mass = 0.0002721, .com{ 0.09, -0.004, 0.00084 } },        // Magnet Cover Screw 2
    CoM{ .mass = 0.00067, .com{ 0.12713, 0.01528, -0.02398 } },     // Tip Ejector Enclosure Screw 1
    CoM{ .mass = 0.00067, .com{ 0.12713, 0.01528, -0.00102 } },     // Tip Ejector Enclosure Screw 2
    CoM{ .mass = 0.00037, .com{ 0.13075, 0, -0.0125 } },            // Ball Bearing (end of lead screw)
    CoM{ .mass = 0.00031, .com{ 0.05076, 0.028, 0.028 } },          // PCB Guide Left Screw 1
    CoM{ .mass = 0.0003, .com{ 0.06319, 0.0295, 0.02106 } },        // PCB Guide Left Screw 2
    CoM{ .mass = 0.0003, .com{ 0.12019, 0.0295, 0.02106 } },        // PCB Guide Left Screw 3
    CoM{ .mass = 0.00031, .com{ 0.05076, -0.028, 0.028 } },         // PCB Guide Rigth Screw 1
    CoM{ .mass = 0.0003, .com{ 0.06319, -0.0295, 0.02106 } },       // PCB Guide Rigth Screw 2
    CoM{ .mass = 0.0003, .com{ 0.12019, -0.0295, 0.02106 } },       // PCB Guide Rigth Screw 3
    CoM{ .mass = 0.00031, .com{ 0.05819, 0.021, 0.01793 } },        // PCB Screw
    CoM{ .mass = 0.00221, .com{ -0.001, 0, -0.025 } },              // Dowel Pin 1
    CoM{ .mass = 0.00221, .com{ -0.001, -0.025, 0 } },              // Dowel Pin 2
    CoM{ .mass = 0.00025, .com{ 0.11833, 0.01258, -0.01248 } },     // Compression Spring
    CoM{ .mass = 0.00997, .com{ 0.08906, 0.00008, 0.01876 } },      // Plain PCB
    CoM{ .mass = 0.0055, .com{ 0.079585, -0.0131, 0.03035 } },      // PCB RS-485
    CoM{ .mass = 0.003, .com{ 0.11949, -0.01992, 0.03035 } },       // PCB Driver Motor + Female header
    CoM{ .mass = 0.0097, .com{ 0.091, 0.01346, 0.03035 } },         // PCB Arduino uC + Female header
    CoM{ .mass = 0.0187317, .com{ 0.08906, 0, 0.01876 } },          // PCB (Remaining)

  };

  auto com = computeCoM(com_vec);

  CoMToInertia(com, inertia.inertia);

  return inertia;
}
