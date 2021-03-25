#include <stdio.h>

#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>
#include <QVBoxLayout>

#include <ros/package.h>

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
    static const boost::filesystem::path pkg_path = ros::package::getPath("rviz_rosbag_player");

    QPixmap play_icon((pkg_path / "icons/media-playback-start.png").c_str());
    QPixmap pause_icon((pkg_path / "icons/media-playback-pause.png").c_str());
    QPixmap eject_icon((pkg_path / "icons/media-eject.png").c_str());

    load_button_ = new QPushButton;
    load_button_->setIcon(QIcon(eject_icon));
    load_button_->setIconSize(eject_icon.size());
    load_button_->setToolTip("load rosbag");
    ctrl_button_ = new QPushButton;
    ctrl_button_->setIcon(play_icon);
    ctrl_button_->setIconSize(play_icon.size());
    ctrl_button_->setToolTip("play/pause");
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

    connect(ctrl_button_, &QPushButton::clicked, [this, play_icon, pause_icon]() {
        if (play_timer_.isActive())
        {
            play_timer_.stop();
            ctrl_button_->setIcon(play_icon);
            ctrl_button_->setIconSize(play_icon.size());
        }
        else
        {
            play_timer_.start();
            ctrl_button_->setIcon(pause_icon);
            ctrl_button_->setIconSize(pause_icon.size());
        }
    });
}

void RosbagPlayer::openBagFile()
{
    bag_file_ =
      QFileDialog::getOpenFileName(this, tr("Open bag"), bag_file_.dir().path(), tr("ROS bag file (*.bag)"));

    if (bag_file_.completeSuffix() != "bag")
    {
        ROS_INFO_STREAM("invalid path=" << bag_file_.absoluteFilePath().toStdString());
        bag_file_ = QFileInfo();
        return;
    }
    ROS_INFO_STREAM("loading file=" << bag_file_.absoluteFilePath().toStdString());
    bag_.close();
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
