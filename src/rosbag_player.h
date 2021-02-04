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

    void loadFile();
    void playCallback();
    void sliderMoved();

protected:
    QPushButton *load_button_, *ctrl_button_;
    QSlider* slider_;
    QFileInfo bag_file_;
    QTimer play_timer_;
    rosbag::Bag bag_;
    std::shared_ptr<rosbag::View> bag_view_;
    rosbag::View::iterator msg_;
    std::map<std::string, ros::Publisher> topic_pub_;
};

} // end namespace rviz_rosbag_player