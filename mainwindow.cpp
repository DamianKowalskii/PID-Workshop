#include "mainwindow.h"

#include "autotuner.h"
#include "autotuning_config.h"
#include "car.h"
#include "car_parameters.h"
#include "pid.h"
#include "population.h"
#include "simulation.h"
#include "simulation_config.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  simulation_timer = new QTimer(this);
  connect(simulation_timer, SIGNAL(timeout()), this, SLOT(simulate()));
  plot_timer = new QTimer(this);
  connect(plot_timer, SIGNAL(timeout()), this, SLOT(update_plot()));
  car = new Model::Car(
      Model::Car_parameters::load, Model::Car_parameters::drag_cefficient,
      Model::Car_parameters::cross_section_area,
      Model::Car_parameters::thrust_parameter, Model::Car_parameters::mass);
  pid = new Model::PID(0, 0, 0, Model::Car_parameters::min_gas_pedal_position,
                       Model::Car_parameters::max_gas_pedal_position);
  autotuner = new Model::Autotuning::Autotuner();
  is_simulation_running = false;
  simulation_time_step_seconds = 1;
  statusBar()->setSizeGripEnabled(false);
  setFixedSize(this->geometry().width(), this->geometry().height());
  init_plots();
  ui->button_stop->setDisabled(true);
  init_gui_strings();
  init_start_values();
}

MainWindow::~MainWindow() {
  delete ui;
  delete simulation_timer;
  delete plot_timer;
  delete car;
  delete pid;
}

void MainWindow::init_gui_strings() {
  this->setWindowTitle(View::Gui::main_window_title);
  ui->label_kp->setText(View::Gui::label_kp);
  ui->label_ki->setText(View::Gui::label_ki);
  ui->label_kd->setText(View::Gui::label_kd);
  ui->label_setpoint->setText(View::Gui::label_setpoint);
  ui->label_simulation_time_step->setText(
      View::Gui::label_simulation_time_step);
  ui->button_start->setText(View::Gui::button_start);
  ui->button_stop->setText(View::Gui::button_stop);
  ui->button_restart->setText(View::Gui::button_restart);
  ui->button_autotune->setText(View::Gui::button_autotune);
  ui->check_box_antiwindup->setText(View::Gui::check_box_enable_anti_windup);
}

void MainWindow::init_start_values() {
  //Blocking signals from ui elements for not triggering callbacks just after starting the program.
    {
    const QSignalBlocker blocker(ui->spin_box_simulation_time_step);
    ui->spin_box_simulation_time_step->setValue(simulation_time_step_seconds);
  }

  {
    const QSignalBlocker blocker(ui->double_spin_box_kp);
    ui->double_spin_box_kp->setValue(pid->get_kp());
  }
  {
    const QSignalBlocker blocker(ui->double_spin_box_ki);
    ui->double_spin_box_ki->setValue(pid->get_ki());
  }
  {
    const QSignalBlocker blocker(ui->double_spin_box_kd);
    ui->double_spin_box_kd->setValue(pid->get_kd());
  }
  {
    const QSignalBlocker blocker(ui->double_spin_box_setpoint);
    ui->double_spin_box_setpoint->setValue(simulation_vars.setpoint);
  }
}

void MainWindow::init_plots() {
  ui->plot_process->xAxis->setLabel(View::Gui::time_axis);
  ui->plot_process->yAxis->setLabel(View::Gui::velocity_axis);
  ui->plot_process->addGraph();
  ui->plot_process->addGraph();
  ui->plot_process->legend->setVisible(true);
  ui->plot_process->graph(View::Gui::setpoint_graph)
      ->setName(View::Gui::setpoint_graph_label);
  ui->plot_process->graph(View::Gui::output_graph)
      ->setName(View::Gui::output_graph_label);
  ui->plot_process->graph(View::Gui::setpoint_graph)
      ->setPen(QPen(Qt::green, 2));
  ui->plot_process->graph(View::Gui::output_graph)->setPen(QPen(Qt::blue, 2));
  ui->plot_pid->xAxis->setLabel(View::Gui::time_axis);
  ui->plot_pid->yAxis->setLabel(View::Gui::cruise_control_axis);
  ui->plot_pid->addGraph();
  ui->plot_pid->legend->setVisible(true);
  ui->plot_pid->graph(View::Gui::pid_output_graph)
      ->setName(View::Gui::pid_output_graph_label);
  ui->plot_pid->graph(View::Gui::pid_output_graph)->setPen(QPen(Qt::red, 2));
}

void MainWindow::restart() {
  simulate();
  update_plot();
  ui->plot_pid->graph(View::Gui::pid_output_graph)->data()->clear();
  ui->plot_process->graph(View::Gui::output_graph)->data()->clear();
  ui->plot_process->graph(View::Gui::setpoint_graph)->data()->clear();
  ui->plot_pid->xAxis->setRange(0, max_starting_range_plot);
  ui->plot_process->xAxis->setRange(0, max_starting_range_plot);
  ui->plot_pid->yAxis->setRange(0, max_starting_range_plot);
  ui->plot_process->yAxis->setRange(0, max_starting_range_plot);
  car->restart();
  pid->restart();
  simulation_vars.accumulate_time = 0;
  simulation_vars.gas_pedal = 0;
  simulation_vars.velocity = 0;
  simulation_sample.car_output_now.first = simulation_vars.accumulate_time;
  simulation_sample.car_output_now.second = simulation_vars.velocity;
  simulation_sample.setpoint_now.first = simulation_vars.accumulate_time;
  simulation_sample.setpoint_now.second = simulation_vars.setpoint;
  simulation_sample.pid_output_now.first = simulation_vars.accumulate_time;
  simulation_sample.pid_output_now.second = simulation_vars.gas_pedal;
}

void MainWindow::update_plot() {
  ui->plot_process->graph(View::Gui::setpoint_graph)
      ->addData(simulation_sample.setpoint_now.first,
                simulation_sample.setpoint_now.second);
  ui->plot_process->graph(View::Gui::output_graph)
      ->addData(simulation_sample.car_output_now.first,
                simulation_sample.car_output_now.second);

  ui->plot_pid->graph(View::Gui::pid_output_graph)
      ->addData(simulation_sample.pid_output_now.first,
                simulation_sample.pid_output_now.second);
  ui->plot_process->graph(View::Gui::output_graph)->rescaleAxes(true);
  ui->plot_process->graph(View::Gui::setpoint_graph)->rescaleAxes(true);
  ui->plot_pid->graph(View::Gui::pid_output_graph)->rescaleAxes(true);

  ui->plot_pid->replot();
  ui->plot_process->replot();
  ui->plot_pid->update();
  ui->plot_process->update();
}

void MainWindow::simulate() {
  double loop_time_ms =
      static_cast<double>(Model::Simulation::pid_loop_time_ms);
  int32_t pid_loops = simulation_time_step_seconds * loop_time_ms;
  for (int i = 0; i < pid_loops; ++i) {
    simulation_vars.gas_pedal = pid->calculate_output(
        simulation_vars.velocity, simulation_vars.setpoint, 1 / loop_time_ms);
    car->set_gas_pedal_position(simulation_vars.gas_pedal);
    car->drive(loop_time_ms);
    simulation_vars.velocity = car->get_velocity();
    simulation_vars.accumulate_time += 1 / loop_time_ms;
  }

  simulation_sample.car_output_now.first = simulation_vars.accumulate_time;
  simulation_sample.car_output_now.second = simulation_vars.velocity;
  simulation_sample.setpoint_now.first = simulation_vars.accumulate_time;
  simulation_sample.setpoint_now.second = simulation_vars.setpoint;
  simulation_sample.pid_output_now.first = simulation_vars.accumulate_time;
  simulation_sample.pid_output_now.second = simulation_vars.gas_pedal;
}

void MainWindow::on_button_start_clicked() {
  simulation_timer->start(simulation_timer_interval_ms);
  plot_timer->start(plot_refresh_rate_miliseconds);
  ui->button_start->setDisabled(true);
  ui->button_stop->setDisabled(false);
  is_simulation_running = true;
}

void MainWindow::on_button_stop_clicked() {
  simulation_timer->stop();
  plot_timer->stop();
  ui->button_stop->setDisabled(true);
  ui->button_start->setDisabled(false);
  is_simulation_running = false;
}

void MainWindow::on_double_spin_box_kp_valueChanged(double arg1) {
  pid->set_kp(arg1);
}

void MainWindow::on_double_spin_box_ki_valueChanged(double arg1) {
  pid->set_ki(arg1);
}

void MainWindow::on_double_spin_box_kd_valueChanged(double arg1) {
  pid->set_kd(arg1);
}

void MainWindow::on_double_spin_box_setpoint_valueChanged(double arg1)

{
  simulation_vars.setpoint = arg1;
  if (simulation_vars.setpoint < 0) {
    simulation_vars.setpoint = 0;
  }
  ui->double_spin_box_setpoint->setValue(simulation_vars.setpoint);
}

void MainWindow::on_check_box_antiwindup_stateChanged(int arg1) {
  bool state = ui->check_box_antiwindup->checkState();

  if (state == true) {
    pid->set_prevent_windup(true);
  } else {
    pid->set_prevent_windup(false);
  }
}

void MainWindow::on_spin_box_simulation_time_step_valueChanged(int arg1) {
  simulation_time_step_seconds = arg1;
}

void MainWindow::on_button_restart_clicked() {
  restart();
  restart();
}

void MainWindow::on_button_autotune_clicked() {
  autotuner->run_autotuning();
  ui->double_spin_box_kp->setValue(autotuner->get_the_best_kp());
  ui->double_spin_box_ki->setValue(autotuner->get_the_best_ki());
  ui->double_spin_box_kd->setValue(autotuner->get_the_best_kd());
}
