/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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

RosbagPlayer::RosbagPlayer(QWidget* parent)
  : rviz::Panel(parent)
{
    load_button_ = new QPushButton("load");
    load_button_->setFixedWidth(100);
    ctrl_button_ = new QPushButton("start/stop");
    ctrl_button_->setFixedWidth(100);
    slider_ = new QSlider(Qt::Horizontal);

    QHBoxLayout* layout = new QHBoxLayout;
    layout->addWidget(load_button_);
    layout->addWidget(ctrl_button_);
    layout->addWidget(slider_);
    setLayout(layout);

    connect(load_button_, &QPushButton::clicked, this, &RosbagPlayer::loadFile);

    connect(ctrl_button_, &QPushButton::clicked, [this]() {
        if (play_timer_.isActive())
        {
            play_timer_.stop();
        }
        else
        {
            play_timer_.start(1000);
        }
    });
}

void RosbagPlayer::loadFile()
{
    bag_file_ =
      QFileDialog::getOpenFileName(this, tr("Open bag"), bag_file_.dir().path(), tr("ROS bag file (*.bag)"));

    ROS_INFO_STREAM("loading file=" << bag_file_.absoluteFilePath().toStdString());
    bag_.close();
    bag_.open(bag_file_.absoluteFilePath().toStdString(), rosbag::bagmode::Read);

    bag_view_ = std::make_shared<rosbag::View>(bag_);
    msg_ = bag_view_->begin();

    connect(&play_timer_, &QTimer::timeout, this, &RosbagPlayer::playCallback);
    connect(&slider_, &QSlider::valueChanged, this, &RosbagPlayer::sliderMoved);
}

void RosbagPlayer::sliderMoved() {}

void RosbagPlayer::playCallback()
{
    if (!bag_view_ or msg_ == bag_view_->end())
    {
        ROS_INFO("reached end.");
        play_timer_.stop();
        return;
    }
    auto pub_it = topic_pub_.find(msg_->getTopic());
    if (pub_it == topic_pub_.end())
    {
        ROS_INFO_STREAM("topic=" << msg_->getTopic());
        ros::NodeHandle nh;
        ros::AdvertiseOptions options(
          msg_->getTopic(), 10, msg_->getMD5Sum(), msg_->getDataType(), msg_->getMessageDefinition());
        topic_pub_[msg_->getTopic()] = nh.advertise(options);
        pub_it = topic_pub_.find(msg_->getTopic());
    }

    pub_it->second.publish(*msg_);
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
