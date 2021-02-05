#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <rviz/panel.h>
#endif

#include <QFileInfo>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>

namespace rviz_rosbag_player
{

class RosbagPlayer : public rviz::Panel
{
    Q_OBJECT
public:
    RosbagPlayer(QWidget* parent = 0);

    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

protected:
    void openBagFile();
    void playCallback();
    void moveBagTo(int new_pos);
    void updateSlider(ros::Time time);

    ros::Publisher& getPublisher(const rosbag::MessageInstance& msg);

protected:
    static const int queue_size_ = 1; // TODO

    QPushButton *load_button_, *ctrl_button_;
    QSlider* slider_;
    QFileInfo bag_file_;
    QTimer play_timer_;
    QLabel* slider_label_;
    rosbag::Bag bag_;
    std::shared_ptr<rosbag::View> bag_view_;
    rosbag::View::iterator msg_, last_msg_;
    ros::WallTime last_pub_time_;
    std::map<std::string, ros::Publisher> topic_pub_;
};

} // end namespace rviz_rosbag_player