The encoder resolution, noise feedback, and backlash output are implemented in the `read` and `write` methods of the `GazeboSystem` class.

1. **Encoder Resolution**: Encoder resolution is implemented in the `read` method. After reading the joint position from the Gazebo simulator, the `round_position` function is called to round the joint position to a certain number of decimal places. This effectively sets the encoder resolution. Here is the relevant code snippet:

```cpp
for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
 if (this->dataPtr->sim_joints_[j]) {
    this->dataPtr->joint_position_[j] = this->dataPtr->sim_joints_[j]->Position(0);

    // This is encoder resolution block
    if (true) 
    { 
      this->dataPtr->round_position(this->dataPtr->joint_position_[j], 2); // Round to 2 decimal places
    }
    ...
 }
}
```

2. **Noise Feedback**: Noise feedback is implemented in the `read` method. The joint velocity and effort readings are augmented with random noise. This simulates the effect of noise in real-world sensor readings. Here is the relevant code snippet:

```cpp
// This is noise feedback block
if (true) 
{ 
 for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
    if (this->dataPtr->sim_joints_[j]) {
      this->dataPtr->joint_velocity_[j] += distribution(generator); // Add noise to velocity
      this->dataPtr->joint_effort_[j] += distribution(generator);   // Add noise to effort
    }
 }
}
```

3. **Backlash Output**: Backlash output is implemented in the `write` method. When setting the joint position, a backlash of 2 units is added to simulate the effect of mechanical backlash in the system. This is done only if a certain condition is met (which is currently commented out in the code). Here is the relevant code snippet:

```cpp
// This block adds the backlash of the system. Somehow it doesn't work.
if (false) {
 double desired_position = this->dataPtr->joint_position_cmd_[j];
 double actual_position = this->dataPtr->sim_joints_[j]->Position(0);
 double position_diff = abs(desired_position - actual_position);
 if (position_diff < 0.01) {
    if (desired_position > actual_position) {
      this->dataPtr->sim_joints_[j]->SetPosition(0, actual_position + 0.01, true);
    } else {
      this->dataPtr->sim_joints_[j]->SetPosition(0, actual_position - 0.01, true);
    }
 } else {
    this->dataPtr->sim_joints_[j]->SetPosition(0, desired_position, true);
 }
}
else
{
 this->dataPtr->sim_joints_[j]->SetPosition(0, this->dataPtr->joint_position_cmd_[j], true);
}
```