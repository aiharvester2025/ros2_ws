#include "oil_palm_harvester_description/range_sensor_panel.hpp"

#include <cmath>

#include <QDockWidget>
#include <QGridLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QTimer>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace oil_palm_harvester_description
{

namespace
{

constexpr std::array<const char *, 5> kSensorNames{
  "Centre", "Left 45 deg", "Right 45 deg", "Left side", "Right side"};
constexpr std::array<const char *, 5> kSensorTopics{
  "/harvester/center_range",
  "/harvester/left_45_range",
  "/harvester/right_45_range",
  "/harvester/left_side_range",
  "/harvester/right_side_range"};
constexpr const char * kCameraSelectionTopic = "/harvester/camera_view/select";

}  // namespace

RangeSensorPanel::RangeSensorPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  setWindowTitle("Docking Sensor Values");
  setMinimumWidth(280);
  setMaximumWidth(340);

  auto * layout = new QGridLayout(this);
  layout->setContentsMargins(10, 8, 10, 10);
  layout->setHorizontalSpacing(14);
  layout->setVerticalSpacing(6);

  auto * title = new QLabel("DOCKING RANGE SENSORS", this);
  title->setStyleSheet("font-weight: 700; color: #f0f0f0;");
  layout->addWidget(title, 0, 0, 1, 2);

  for (std::size_t index = 0; index < kSensorNames.size(); ++index) {
    auto * name = new QLabel(kSensorNames[index], this);
    name->setStyleSheet("color: #d5d5d5;");
    layout->addWidget(name, static_cast<int>(index + 1), 0);

    auto * value = new QLabel("Waiting for data", this);
    value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    value->setStyleSheet("font-weight: 600; color: #f2ad32;");
    layout->addWidget(value, static_cast<int>(index + 1), 1);
    value_labels_[index] = value;
  }

  auto * note = new QLabel("Sensor range: 0.05–3.00 m", this);
  note->setStyleSheet("color: #9fa6ad; font-size: 10px;");
  layout->addWidget(note, 6, 0, 1, 2);

  auto * camera_title = new QLabel("CAMERA VIEW", this);
  camera_title->setStyleSheet("font-weight: 700; color: #f0f0f0; margin-top: 8px;");
  layout->addWidget(camera_title, 7, 0, 1, 2);

  cutter_camera_button_ = new QPushButton("Cutter camera", this);
  cutter_camera_button_->setCheckable(true);
  docking_camera_button_ = new QPushButton("Docking camera", this);
  docking_camera_button_->setCheckable(true);
  layout->addWidget(cutter_camera_button_, 8, 0);
  layout->addWidget(docking_camera_button_, 8, 1);

  camera_status_label_ = new QLabel("Selected: Cutter camera", this);
  camera_status_label_->setStyleSheet("color: #9fa6ad; font-size: 10px;");
  layout->addWidget(camera_status_label_, 9, 0, 1, 2);

  setStyleSheet(
    "RangeSensorPanel { background: #252a31; }"
    "QLabel { background: transparent; }"
    "QPushButton { border: 1px solid #4d5864; border-radius: 3px; padding: 5px; "
    "color: #dfe7ef; background: #303842; }"
    "QPushButton:hover { background: #3d4855; }"
    "QPushButton:checked { border: 1px solid #7de39d; color: #11181e; "
    "background: #7de39d; font-weight: 700; }");

  connect(
    this, &RangeSensorPanel::readingReceived,
    this, &RangeSensorPanel::setReading,
    Qt::QueuedConnection);
  connect(
    cutter_camera_button_, &QPushButton::clicked,
    this, &RangeSensorPanel::selectCutterCamera);
  connect(
    docking_camera_button_, &QPushButton::clicked,
    this, &RangeSensorPanel::selectDockingCamera);

  // The selector itself defaults to the cutter camera.  Reflect that default
  // in the panel immediately, including before RViz has a ROS node.
  cutter_camera_button_->setChecked(true);
}

void RangeSensorPanel::onInitialize()
{
  const auto ros_node_abstraction =
    getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    for (auto * label : value_labels_) {
      label->setText("RViz node unavailable");
      label->setStyleSheet("font-weight: 600; color: #ee6a5f;");
    }
    return;
  }

  const auto node = ros_node_abstraction->get_raw_node();
  rclcpp::QoS selection_qos(1);
  selection_qos.reliable().transient_local();
  camera_selection_publisher_ = node->create_publisher<std_msgs::msg::String>(
    kCameraSelectionTopic, selection_qos);

  for (std::size_t index = 0; index < kSensorTopics.size(); ++index) {
    range_subscriptions_[index] = node->create_subscription<sensor_msgs::msg::Range>(
      kSensorTopics[index], rclcpp::SensorDataQoS(),
      [this, index](const sensor_msgs::msg::Range::SharedPtr message) {
        bool in_range = false;
        const auto text = formatReading(*message, in_range);
        Q_EMIT readingReceived(static_cast<int>(index), text, in_range);
      });
  }
  // Publish the default after the transient-local publisher exists, so a
  // selector started before or after RViz receives the same safe default.
  selectCutterCamera();

  // Panels are created by RViz as dock widgets.  Move this compact status
  // panel to the right-hand dock once the parent widget has been installed.
  QTimer::singleShot(250, this, &RangeSensorPanel::moveToRightDock);
}

void RangeSensorPanel::selectCutterCamera()
{
  setCameraSelection("cutter");
}

void RangeSensorPanel::selectDockingCamera()
{
  setCameraSelection("docking");
}

void RangeSensorPanel::setCameraSelection(const char * selection)
{
  const bool cutter_selected = QString::fromLatin1(selection) == "cutter";
  if (cutter_camera_button_ != nullptr) {
    cutter_camera_button_->setChecked(cutter_selected);
  }
  if (docking_camera_button_ != nullptr) {
    docking_camera_button_->setChecked(!cutter_selected);
  }
  if (camera_status_label_ != nullptr) {
    camera_status_label_->setText(
      cutter_selected ? "Selected: Cutter camera" : "Selected: Docking camera");
  }

  if (!camera_selection_publisher_) {
    return;
  }
  std_msgs::msg::String message;
  message.data = selection;
  camera_selection_publisher_->publish(message);
}

QString RangeSensorPanel::formatReading(
  const sensor_msgs::msg::Range & message, bool & in_range)
{
  in_range = std::isfinite(message.range) &&
    message.range < message.max_range - 0.002F;
  if (in_range) {
    return QString::number(message.range, 'f', 2) + " m";
  }
  if (std::isfinite(message.max_range) && message.max_range > 0.0F) {
    return "> " + QString::number(message.max_range, 'f', 2) + " m";
  }
  return "Out of range";
}

void RangeSensorPanel::setReading(int index, const QString & text, bool in_range)
{
  if (index < 0 || index >= static_cast<int>(value_labels_.size())) {
    return;
  }
  value_labels_[static_cast<std::size_t>(index)]->setText(text);
  value_labels_[static_cast<std::size_t>(index)]->setStyleSheet(
    in_range ? "font-weight: 700; color: #7de39d;" :
    "font-weight: 600; color: #b8c0c8;");
}

void RangeSensorPanel::moveToRightDock()
{
  QDockWidget * dock_widget = nullptr;
  for (QWidget * widget = parentWidget(); widget != nullptr && dock_widget == nullptr;
    widget = widget->parentWidget())
  {
    dock_widget = qobject_cast<QDockWidget *>(widget);
  }
  auto * main_window = dock_widget ?
    qobject_cast<QMainWindow *>(dock_widget->window()) : nullptr;
  if (dock_widget != nullptr && main_window != nullptr) {
    main_window->addDockWidget(Qt::RightDockWidgetArea, dock_widget);
    dock_widget->resize(270, dock_widget->height());
  }
}

}  // namespace oil_palm_harvester_description

PLUGINLIB_EXPORT_CLASS(
  oil_palm_harvester_description::RangeSensorPanel,
  rviz_common::Panel)
