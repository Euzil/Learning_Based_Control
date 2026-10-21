#include "simulate/glfw_adapter.h"

// forward declaration sim manager
class SimManager;

// Custom Adapter to intercept the main UI thread's render loop
class RosGlfwAdapter : public mujoco::GlfwAdapter
{
    public:
        RosGlfwAdapter(SimManager* manager);

        void SwapBuffers() override;

    private:
        SimManager* manager_;
};