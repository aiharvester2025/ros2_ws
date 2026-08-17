#ifndef OIL_PALM_HARVESTER_DESCRIPTION__RANGE_SENSOR_PANEL_HPP_
#define OIL_PALM_HARVESTER_DESCRIPTION__RANGE_SENSOR_PANEL_HPP_

#include <array>
#include <memory>

#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/string.hpp>

class QLabel;
class QPushButton;

namespace oil_palm_harvester_description
{

/// A compact, fixed-screen RViz panel for the five docking range sensors and
/// one cutter-attached clearance sensor.
///
/// Unlike 3-D text markers, these readings stay in one readable location
/// while the camera, C-channel and platform move.
class RangeSensorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RangeSensorPanel(QWidget * parent = nullptr);
  ~RangeSensorPanel() override = default;

  void onInitialize() override;

Q_SIGNALS:
  void readingReceived(int index, const QString & text, bool in_range);
  void cutterLeftReadingReceived(const QString & text, bool in_range);

private Q_SLOTS:
  void setReading(int index, const QString & text, bool in_range);
  void setCutterLeftReading(const QString & text, bool in_range);
  void selectCutterCamera();
  void selectDockingCamera();

private:
  void setCameraSelection(const char * selection);
  void moveToRightDock();
  static QString formatReading(const sensor_msgs::msg::Range & message, bool & in_range);

  std::array<QLabel *, 5> value_labels_{};
  QLabel * cutter_left_value_label_{nullptr};
  QLabel * camera_status_label_{nullptr};
  QPushButton * cutter_camera_button_{nullptr};
  QPushButton * docking_camera_button_{nullptr};
  std::array<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr, 5>
    range_subscriptions_{};
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr cutter_left_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_selection_publisher_;
};

}  // namespace oil_palm_harvester_description

#endif  // OIL_PALM_HARVESTER_DESCRIPTION__RANGE_SENSOR_PANEL_HPP_
