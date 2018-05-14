// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <seat_car_controller.h>

namespace effort_controllers
{
  /**
   * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and sends
   * commands to a \b position interface.
   */
  typedef seat_car_controller::SeatCarController<hardware_interface::EffortJointInterface> SeatCarController;
}

PLUGINLIB_EXPORT_CLASS(effort_controllers::SeatCarController, controller_interface::ControllerBase)
