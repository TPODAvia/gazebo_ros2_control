To add feedback noise and joint backlash to the joints in your code, you need to modify the `GazeboSystem` class. Specifically, you would need to adjust how the joint position, velocity, and effort are read and written. 

Feedback noise can be added by adding a random value to the joint position, velocity, and effort values during the `read` function. Joint backlash can be added by limiting the change in joint position during the `write` function.

Here is an example of how you could implement these changes:

```cpp
// Include the necessary libraries for generating random numbers and limiting values
#include <random>
#include <algorithm>

// ...

// Define a global variable for the random number generator
std::default_random_engine generator;
std::uniform_real_distribution<double> distribution(-0.01, 0.01); // Adjust range as needed

// ...

hardware_interface::return_type GazeboSystem::read(
 const rclcpp::Time & time,
 const rclcpp::Duration & period)
{
 // ...
 
 for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
   if (this->dataPtr->sim_joints_[j]) {
     this->dataPtr->joint_position_[j] += distribution(generator); // Add noise to position
     this->dataPtr->joint_velocity_[j] += distribution(generator); // Add noise to velocity
     this->dataPtr->joint_effort_[j] += distribution(generator);   // Add noise to effort
   }
 }

 // ...
}

hardware_interface::return_type GazeboSystem::write(
 const rclcpp::Time & time,
 const rclcpp::Duration & period)
{
 // ...

 for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
   if (this->dataPtr->sim_joints_[j]) {
     // Limit change in position
     double delta_position = this->dataPtr->joint_position_cmd_[j] - this->dataPtr->joint_position_[j];
     if (delta_position > 0.1) { // Adjust threshold as needed
       this->dataPtr->joint_position_cmd_[j] -= delta_position - 0.1;
     } else if (delta_position < -0.1) {
       this->dataPtr->joint_position_cmd_[j] -= delta_position + 0.1;
     }
     
     // ...
   }
 }

 // ...
}
```

In the `read` function, a random value between -0.01 and 0.01 is added to the joint position, velocity, and effort values to simulate feedback noise. The `write` function limits the change in joint position to a maximum of 0.1 units to simulate joint backlash. You can adjust these values as needed to match the characteristics of your robot.

Please note that this is a simple implementation and might not cover all scenarios. For example, the noise and backlash models are linear and constant over time, which might not accurately represent real-world behavior. Also, the noise is added uniformly to all joints without considering their individual characteristics. You might want to consider these factors when implementing a more realistic simulation.