#ifndef GUI_H
#define GUI_H
#include <QString>
namespace View {
namespace Gui {
static const QString main_window_title = "PID Workshop";
static const QString label_kp = "Kp";
static const QString label_ki = "Ki";
static const QString label_kd = "Kd";
static const QString label_setpoint = "Setpoint";
static const QString label_simulation_time_step = "Simulation Timestep (s)";
static const QString label_load = "Load";
static const QString button_start = "Start";
static const QString button_stop = "Stop";
static const QString button_restart = "Restart";
static const QString button_autotune = "Start autotuning";
static const QString check_box_enable_anti_windup = "Enable Anti-windup";
static const QString pid_output_graph_label = "PID output";
static const QString setpoint_graph_label = "Setpoint";
static const QString output_graph_label = "Output";
static const QString time_axis = "Time (s)";
static const QString velocity_axis = "Velocity (m/s)";
static const QString cruise_control_axis = "Braking/Acceleration (scalar)";
static const uint32_t output_graph = 0;
static const uint32_t setpoint_graph = 1;
static const uint32_t pid_output_graph = 0;

}  // namespace Gui
}  // namespace View
#endif
