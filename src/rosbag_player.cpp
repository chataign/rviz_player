#include <stdio.h>

#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>

#include "rosbag_player.h"

namespace rviz_rosbag_player
{
QString formatDuration(ros::Duration duration)
{
    int hours = duration.toSec() / 3600;
    int mins = (duration.toSec() - hours * 3600) / 60;
    int secs = duration.toSec() - hours * 3600 - mins * 60;

    char time_str[20];
    snprintf(time_str, 20, "%02d:%02d:%02d", hours, mins, secs);
    return QString::fromStdString(time_str);
}

RosbagPlayer::RosbagPlayer(QWidget* parent)
  : rviz::Panel(parent)
{
    load_button_ = new QPushButton("load");
    load_button_->setFixedWidth(100);
    ctrl_button_ = new QPushButton("start/stop");
    ctrl_button_->setFixedWidth(100);
    slider_ = new QSlider(Qt::Horizontal);
    slider_->setTracking(false);
    slider_->setMinimum(0);
    slider_->setMaximum(1000);
    slider_label_ = new QLabel;
    slider_label_->setFixedWidth(80);

    play_timer_.setInterval(10);

    QHBoxLayout* layout = new QHBoxLayout;
    layout->addWidget(load_button_);
    layout->addWidget(ctrl_button_);
    layout->addWidget(slider_);
    layout->addWidget(slider_label_);
    setLayout(layout);

    connect(slider_, &QSlider::sliderPressed, [this]() { play_timer_.stop(); });
    connect(slider_, &QSlider::sliderReleased, [this]() { play_timer_.start(); });
    connect(load_button_, &QPushButton::clicked, this, &RosbagPlayer::openBagFile);
    connect(&play_timer_, &QTimer::timeout, this, &RosbagPlayer::playCallback);
    connect(slider_, &QSlider::valueChanged, this, &RosbagPlayer::moveBagTo);

    connect(ctrl_button_, &QPushButton::clicked, [this]() {
        play_timer_.isActive() ? play_timer_.stop() : play_timer_.start();
    });
}

void RosbagPlayer::openBagFile()
{
    bag_file_ =
      QFileDialog::getOpenFileName(this, tr("Open bag"), bag_file_.dir().path(), tr("ROS bag file (*.bag)"));

    ROS_INFO_STREAM("loading file=" << bag_file_.absoluteFilePath().toStdString());
    bag_.open(bag_file_.absoluteFilePath().toStdString(), rosbag::bagmode::Read);
    bag_view_ = std::make_shared<rosbag::View>(bag_);
    ROS_INFO("bag duration=%.0fs", (bag_view_->getEndTime() - bag_view_->getBeginTime()).toSec());
    msg_ = bag_view_->begin();
    last_msg_ = msg_;
}

void RosbagPlayer::moveBagTo(int new_pos)
{
    if (!bag_view_)
    {
        return;
    }

    float new_percent = (new_pos - slider_->minimum()) / float(slider_->maximum() - slider_->minimum());
    ROS_INFO("slider moved: pos=%d percent=%.1f", new_pos, new_percent);

    ros::Time new_time =
      bag_view_->getBeginTime() +
      ros::Duration(new_percent * (bag_view_->getEndTime() - bag_view_->getBeginTime()).toSec());

    if (msg_ == bag_view_->end() or new_time < msg_->getTime())
    {
        msg_ = bag_view_->begin();
    }

    ROS_INFO("new time=%.2f", new_time.toSec());

    while (msg_->getTime() < new_time)
    {
        ++msg_;
    }

    last_msg_ = msg_;
    updateSlider(msg_->getTime());
}

void RosbagPlayer::updateSlider(ros::Time new_time)
{
    if (!bag_view_)
    {
        return;
    }

    float bag_percent = (new_time - bag_view_->getBeginTime()).toSec() /
                        (bag_view_->getEndTime() - bag_view_->getBeginTime()).toSec();

    int slider_pos = slider_->minimum() + bag_percent * (slider_->maximum() - slider_->minimum());
    // ROS_INFO("moving slider to percent=%.4f pos=%d", bag_percent, slider_pos);

    slider_->setSliderPosition(slider_pos);
    slider_label_->setText(formatDuration(new_time - bag_view_->getBeginTime()));
}

ros::Publisher& RosbagPlayer::getPublisher(const rosbag::MessageInstance& msg)
{
    auto pub_it = topic_pub_.find(msg.getTopic());

    if (pub_it == topic_pub_.end())
    {
        ROS_INFO_STREAM("publishing topic=" << msg.getTopic());
        ros::NodeHandle nh;
        ros::AdvertiseOptions options(
          msg.getTopic(), queue_size_, msg.getMD5Sum(), msg.getDataType(), msg.getMessageDefinition());
        topic_pub_[msg.getTopic()] = nh.advertise(options);
        pub_it = topic_pub_.find(msg.getTopic());
    }

    return pub_it->second;
}

void RosbagPlayer::playCallback()
{
    if (!bag_view_)
    {
        return;
    }
    if (msg_ == bag_view_->end())
    {
        ROS_INFO("reached end.");
        play_timer_.stop();
        return;
    }

    auto time_now = ros::WallTime::now();

    if (last_msg_ != bag_view_->end())
    {
        ros::WallDuration interval((msg_->getTime() - last_msg_->getTime()).toSec());

        if (last_pub_time_ + interval > time_now)
        {
            ros::WallDuration sleep_time = (last_pub_time_ + interval) - time_now;
            // ROS_INFO("sleep=%.4fs", sleep_time.toSec());
            sleep_time.sleep();
        }
    }

    auto& pub = getPublisher(*msg_);
    pub.publish(*msg_);
    updateSlider(msg_->getTime());
    last_msg_ = msg_;
    last_pub_time_ = time_now;
    ++msg_;
}

void RosbagPlayer::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    config.mapSetValue("BagFile", bag_file_.absoluteFilePath());
}

void RosbagPlayer::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
    QString bag_file;
    if (config.mapGetString("BagFile", &bag_file))
    {
        bag_file_ = bag_file;
        ROS_INFO_STREAM("bag_file=" << bag_file.toStdString());
    }
}

} // end namespace rviz_rosbag_player

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_rosbag_player::RosbagPlayer, rviz::Panel)
