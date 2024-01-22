To add position feedback resolution caused by using encoders to the joints, you need to modify the `read` function in the `GazeboSystem` class. Currently, the `read` function is getting the joint positions, velocities, and efforts from the simulation model. To add encoder feedback, you would need to read the encoder values from the simulation model and store them in the `joint_position_` vector.

Here is how you could modify the `read` function:

```cpp
hardware_interface::return_type GazeboSystem::read(
 const rclcpp::Time & time,
 const rclcpp::Duration & period)
{
 for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
   if (this->dataPtr->sim_joints_[j]) {
     // Get the position from the encoder
     this->dataPtr->joint_position_[j] = this->dataPtr->sim_joints_[j]->EncoderPosition(0);
     this->dataPtr->joint_velocity_[j] = this->dataPtr->sim_joints_[j]->GetVelocity(0);
     this->dataPtr->joint_effort_[j] = this->dataPtr->sim_joints_[j]->GetForce(0u);
   }
 }

 // Rest of the function...
}
```
In the above code, replace `EncoderPosition(0)` with the appropriate method call to get the encoder position from your simulation model. The exact method will depend on the API of your simulation model.

Please note that the `read` function is called at every update cycle of the simulation. Therefore, the encoder values will be updated frequently, providing real-time feedback.

Remember to test your changes thoroughly to ensure that the encoder feedback is working correctly and providing accurate position feedback.