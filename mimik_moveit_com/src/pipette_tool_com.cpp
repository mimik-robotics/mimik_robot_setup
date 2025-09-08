#include <mimik_moveit_com/com.h>

using namespace mimik_moveit_com;

int main(int argc, char** argv)
{
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

  print(com);
}
