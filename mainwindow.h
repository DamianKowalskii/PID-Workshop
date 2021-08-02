#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "car.h"
#include "gui.h"
#include "pid.h"
#include "simulation.h"
#include "autotuner.h"
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

 private:
  void init_start_values();
  void init_plots();
  void plot_process();
  void init_gui_strings();
  void restart();
 public slots:
  void update_plot();
  void simulate();
 private slots:
  void on_button_start_clicked();

  void on_button_stop_clicked();

  void on_double_spin_box_kp_valueChanged(double arg1);

  void on_double_spin_box_ki_valueChanged(double arg1);

  void on_double_spin_box_kd_valueChanged(double arg1);

  void on_double_spin_box_setpoint_valueChanged(double arg1);

  void on_check_box_antiwindup_stateChanged(int arg1);

  void on_spin_box_simulation_time_step_valueChanged(int arg1);

  void on_button_restart_clicked();

  void on_button_autotune_clicked();

 private:
  Ui::MainWindow *ui;
  QTimer *simulation_timer;
  QTimer *plot_timer;
  Model::Autotuning::Autotuner *autotuner;
  Model::Car *car;
  Model::PID *pid;
  Model::Simulation::simulation_vars_t simulation_vars;
  Model::Simulation::simulation_sample_t simulation_sample;
  static const uint32_t max_starting_range_plot = 5;
  bool is_simulation_running;
  uint32_t simulation_time_step_seconds;
  const uint32_t simulation_timer_interval_ms = 200;
  const uint32_t plot_refresh_rate_miliseconds = 0;
};
#endif  // MAINWINDOW_H
