#include <mujoco_simulation/ros_glfw_adapter.h>
#include <mujoco_simulation/sim_manager.h>

// Pass a pointer to your SimManager so we can call back into it
RosGlfwAdapter::RosGlfwAdapter(SimManager* manager) : manager_(manager)
{
}

void RosGlfwAdapter::SwapBuffers()
{
    // Render and publish the ROS cameras here.
    manager_->renderRosCameras(&this->mjr_context());

    // Call the base class to actually swap the window buffers for the UI
    mujoco::GlfwAdapter::SwapBuffers();
}