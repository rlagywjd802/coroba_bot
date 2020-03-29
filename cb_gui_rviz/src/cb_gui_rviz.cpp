/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik Consulting
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Consulting nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <cstdio>
#include <string>
#include <vector>

#include <QGroupBox>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QFrame>
#include <QMessageBox>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_srvs/Trigger.h>
#include <pc_to_mesh/pc_to_mesh.h>
#include <cvrg_path/cvrg_plan.h>

#include "cb_gui_rviz.h"

namespace cb_gui_rviz
{
CBGuiRviz::CBGuiRviz(QWidget* parent) : rviz::Panel(parent)
{
  //////////////////////////////
  // Button
  //////////////////////////////
  
  // Create a push button
  btn_mb_cancel_ = new QPushButton(this);
  btn_mb_cancel_->setText("Cancel motion planner");
  connect(btn_mb_cancel_, SIGNAL(clicked()), this, SLOT(cancelMB()));

  // Create a push button
  btn_rtabmap_pause_ = new QPushButton(this);
  btn_rtabmap_pause_->setText("Pause");
  connect(btn_rtabmap_pause_, SIGNAL(clicked()), this, SLOT(pauseRtabmap()));

  // Create a push button
  btn_rtabmap_resume_ = new QPushButton(this);
  btn_rtabmap_resume_->setText("Resume");
  connect(btn_rtabmap_resume_, SIGNAL(clicked()), this, SLOT(resumeRtabmap()));

  // Create a push button
  btn_pcl_capture_ = new QPushButton(this);
  btn_pcl_capture_->setText("Capture");
  connect(btn_pcl_capture_, SIGNAL(clicked()), this, SLOT(pclCapture()));

  // Create a push button
  btn_pcl_reset_ = new QPushButton(this);
  btn_pcl_reset_->setText("Clear");
  connect(btn_pcl_reset_, SIGNAL(clicked()), this, SLOT(pclReset()));

  // Create a push button
  btn_abb_trim_ = new QPushButton(this);
  btn_abb_trim_->setText("Trim");
  connect(btn_abb_trim_, SIGNAL(clicked()), this, SLOT(abbTrim()));

  // Create a push button
  btn_abb_reset_ = new QPushButton(this);
  btn_abb_reset_->setText("Reset");
  connect(btn_abb_reset_, SIGNAL(clicked()), this, SLOT(abbReset()));

  btn_abb_save_ = new QPushButton(this);
  btn_abb_save_->setText("Save");
  connect(btn_abb_save_, SIGNAL(clicked()), this, SLOT(abbSave()));

  // Create a push button
  btn_cvrg_plan_ = new QPushButton(this);
  btn_cvrg_plan_->setText("Plan");
  connect(btn_cvrg_plan_, SIGNAL(clicked()), this, SLOT(cvrgPlan()));

  // Create a push button
  btn_scanning_plan_ = new QPushButton(this);
  btn_scanning_plan_->setText("Plan Scanning Motion");
  connect(btn_scanning_plan_, SIGNAL(clicked()), this, SLOT(scanPlan()));

  btn_scanning_execute_ = new QPushButton(this);
  btn_scanning_execute_->setText("Execute Scanning Motion");
  connect(btn_scanning_execute_, SIGNAL(clicked()), this, SLOT(scanExecute()));

  // Create a push button
  btn_gripper_open_ = new QPushButton(this);
  btn_gripper_open_->setText("Open");
  connect(btn_gripper_open_, SIGNAL(clicked()), this, SLOT(moveGripperOpen()));

  // Create a push button
  btn_gripper_close_ = new QPushButton(this);
  btn_gripper_close_->setText("Close");
  connect(btn_gripper_close_, SIGNAL(clicked()), this, SLOT(moveGripperClose()));

  // Create a push button
  btn_appr_plan_ = new QPushButton(this);
  btn_appr_plan_->setText("Plan");
  connect(btn_appr_plan_, SIGNAL(clicked()), this, SLOT(apprPlan()));

  // Create a push button
  btn_appr_execute_ = new QPushButton(this);
  btn_appr_execute_->setText("Execute");
  connect(btn_appr_execute_, SIGNAL(clicked()), this, SLOT(apprExecute()));

  // Create a push button
  btn_retr_plan_ = new QPushButton(this);
  btn_retr_plan_->setText("Plan");
  connect(btn_retr_plan_, SIGNAL(clicked()), this, SLOT(retrPlan()));

  // Create a push button
  btn_retr_execute_ = new QPushButton(this);
  btn_retr_execute_->setText("Execute");
  connect(btn_retr_execute_, SIGNAL(clicked()), this, SLOT(retrExecute()));

  // Create a push button
  btn_move_xp_ = new QPushButton(this);
  btn_move_xp_->setText("Forward");
  connect(btn_move_xp_, SIGNAL(clicked()), this, SLOT(moveXP()));

  // Create a push button
  btn_move_xm_ = new QPushButton(this);
  btn_move_xm_->setText("Backward");
  connect(btn_move_xm_, SIGNAL(clicked()), this, SLOT(moveXM()));

  // Create a push button
  btn_move_yp_ = new QPushButton(this);
  btn_move_yp_->setText("Left");
  connect(btn_move_yp_, SIGNAL(clicked()), this, SLOT(moveYP()));

  // Create a push button
  btn_move_ym_ = new QPushButton(this);
  btn_move_ym_->setText("Right");
  connect(btn_move_ym_, SIGNAL(clicked()), this, SLOT(moveYM()));

  // Create a push button
  btn_move_zp_ = new QPushButton(this);
  btn_move_zp_->setText("Up");
  connect(btn_move_zp_, SIGNAL(clicked()), this, SLOT(moveZP()));

  // Create a push button
  btn_move_zm_ = new QPushButton(this);
  btn_move_zm_->setText("Down");
  connect(btn_move_zm_, SIGNAL(clicked()), this, SLOT(moveZM()));

  // Create a radio button
  rbtn_appr_1_ = new QRadioButton("roll(red)", this);
  rbtn_appr_2_ = new QRadioButton("pitch(green)", this);
  rbtn_appr_3_ = new QRadioButton("yaw(blue)", this);
  connect(rbtn_appr_1_, SIGNAL(clicked()), this, SLOT(apprRB1()));  
  connect(rbtn_appr_2_, SIGNAL(clicked()), this, SLOT(apprRB2()));  
  connect(rbtn_appr_3_, SIGNAL(clicked()), this, SLOT(apprRB3()));
  
  // Create a slider
  slider_ = new QSlider(Qt::Horizontal, this);
  slider_->setMinimum(0);
  slider_->setMaximum(20);
  slider_->setValue(10);
  connect(slider_, SIGNAL(valueChanged(int)), this, SLOT(setDistance(int)));

  // Create a push button
  btn_imarker_clear_ = new QPushButton(this);
  btn_imarker_clear_->setText("Clear");
  connect(btn_imarker_clear_, SIGNAL(clicked()), this, SLOT(clearIMarker()));

  // Create a radio button
  rbtn_motion_1_ = new QRadioButton("pick up", this);
  rbtn_motion_2_ = new QRadioButton("place", this);
  connect(rbtn_motion_1_, SIGNAL(clicked()), this, SLOT(motionRB1()));  
  connect(rbtn_motion_2_, SIGNAL(clicked()), this, SLOT(motionRB2()));

  // Create a text browser
  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(updateText()));
  connect(timer_, SIGNAL(timeout()), this, SLOT(updateMBResult()));
  timer_->start(100);

  text_browser_ = new QTextBrowser(this);
  text_browser_->setStyleSheet("font: 15pt");

  // Create a combo box
  combo_box_ = new QComboBox(this);
  combo_box_->addItem("-----");
  combo_box_->addItem("sol 0");
  combo_box_->addItem("sol 1");
  combo_box_->addItem("sol 2");
  combo_box_->addItem("sol 3");
  combo_box_->addItem("sol 4");
  combo_box_->addItem("sol 5");
  combo_box_->addItem("sol 6");
  combo_box_->addItem("sol 7");
  connect(combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(selectSolution(int)));


  //////////////////////////////
  // Layout
  //////////////////////////////

  // Main Layout
  layout = new QVBoxLayout;

  // text browser
  layout->addWidget(text_browser_);

  // Mobile Base
  lb_mb_ = new QLabel(QString::fromStdString("Mobile Base"));
  lb_mb_->setAlignment(Qt::AlignCenter);
  layout->addWidget(lb_mb_);
  layout->addWidget(btn_mb_cancel_);

  QHBoxLayout* l_rtabmap = new QHBoxLayout;
  lb_mb_loc_ = new QLabel(QString::fromStdString("localization:"));
  l_rtabmap->addWidget(lb_mb_loc_);
  l_rtabmap->addWidget(btn_rtabmap_pause_);
  l_rtabmap->addWidget(btn_rtabmap_resume_);
  layout->addLayout(l_rtabmap);

  // Camera on Manipulator
  lb_pcl_ = new QLabel(QString::fromStdString("Camera on Manipulator"));
  lb_pcl_->setAlignment(Qt::AlignCenter);
  QHBoxLayout* l_pcl = new QHBoxLayout;
  l_pcl->addWidget(btn_pcl_capture_);
  l_pcl->addWidget(btn_pcl_reset_);
  QHBoxLayout* l_abb = new QHBoxLayout;
  l_abb->addWidget(btn_abb_trim_);
  l_abb->addWidget(btn_abb_reset_);
  // l_abb->addWidget(btn_abb_save_);
  layout->addWidget(lb_pcl_);
  layout->addLayout(l_pcl);
  layout->addLayout(l_abb);
  layout->addWidget(btn_cvrg_plan_);
  layout->addWidget(btn_scanning_plan_);
  layout->addWidget(btn_scanning_execute_);

  // Settings for Approaching Pickup
  lb_appr_ = new QLabel(QString::fromStdString("Settings for Approaching Pickup"));
  lb_appr_->setAlignment(Qt::AlignCenter);
  layout->addWidget(lb_appr_);

  QHBoxLayout* l_appr_pose = new QHBoxLayout;
  lb_appr_pose_ = new QLabel(QString::fromStdString("set pre-grasp pose"));
  l_appr_pose->addWidget(rbtn_appr_1_);
  l_appr_pose->addWidget(rbtn_appr_2_);
  l_appr_pose->addWidget(rbtn_appr_3_);  
  layout->addWidget(lb_appr_pose_);
  layout->addLayout(l_appr_pose);

  QHBoxLayout* l_appr_dist = new QHBoxLayout;
  lb_appr_dist_ = new QLabel(QString::fromStdString("displacement"));
  l_appr_dist->addWidget(lb_appr_dist_);
  l_appr_dist->addWidget(slider_);
  l_appr_dist->addWidget(btn_imarker_clear_);
  layout->addLayout(l_appr_dist);

  QHBoxLayout* l_appr_ik = new QHBoxLayout;
  lb_appr_ik_ = new QLabel(QString::fromStdString("select arm configuration"));
  l_appr_ik->addWidget(lb_appr_ik_);
  l_appr_ik->addWidget(combo_box_);
  layout->addLayout(l_appr_ik);

  // Manipulator Motion
  lb_motion_ = new QLabel(QString::fromStdString("Manipulator Motion"));
  lb_motion_->setAlignment(Qt::AlignCenter);
  QHBoxLayout* l_motion_ = new QHBoxLayout;
  // l_motion_->setAlignment(Qt::AlignCenter);
  l_motion_->addWidget(rbtn_motion_1_);
  l_motion_->addWidget(rbtn_motion_2_);
  layout->addWidget(lb_motion_);
  layout->addLayout(l_motion_);

  QHBoxLayout* l_motion_plan_ = new QHBoxLayout;
  QVBoxLayout* l_motion_appr_ = new QVBoxLayout;
  lb_motion_appr_ = new QLabel(QString::fromStdString("Approach"));
  lb_motion_appr_->setAlignment(Qt::AlignCenter);
  l_motion_appr_->addWidget(lb_motion_appr_);
  l_motion_appr_->addWidget(btn_appr_plan_);
  l_motion_appr_->addWidget(btn_appr_execute_);
  QVBoxLayout* l_motion_retr_ = new QVBoxLayout;
  lb_motion_retr_ = new QLabel(QString::fromStdString("Retreat"));
  lb_motion_retr_->setAlignment(Qt::AlignCenter);
  l_motion_retr_->addWidget(lb_motion_retr_);
  l_motion_retr_->addWidget(btn_retr_plan_);
  l_motion_retr_->addWidget(btn_retr_execute_);
  l_motion_plan_->addLayout(l_motion_appr_);
  l_motion_plan_->addLayout(l_motion_retr_);
  layout->addLayout(l_motion_plan_);

  // Move End-Effector
  lb_ee_ = new QLabel(QString::fromStdString("Move End-Effector"));
  lb_ee_->setAlignment(Qt::AlignCenter);
  // QHBoxLayout* l_ee = new QHBoxLayout;
  // QVBoxLayout* l_ee_x = new QVBoxLayout;
  // QVBoxLayout* l_ee_y = new QVBoxLayout;
  // QVBoxLayout* l_ee_z = new QVBoxLayout;
  // l_ee_x->addWidget(btn_move_xp_);
  // l_ee_x->addWidget(btn_move_xm_);
  // l_ee_y->addWidget(btn_move_yp_);
  // l_ee_y->addWidget(btn_move_ym_);
  // l_ee_z->addWidget(btn_move_zp_);
  // l_ee_z->addWidget(btn_move_zm_);
  // l_ee->addLayout(l_ee_x);
  // l_ee->addLayout(l_ee_y);
  // l_ee->addLayout(l_ee_z);
  QHBoxLayout* l_ee_z = new QHBoxLayout;
  l_ee_z->addWidget(btn_move_zp_);
  l_ee_z->addWidget(btn_move_zm_);
  QGridLayout* l_ee = new QGridLayout();
  l_ee->addWidget(btn_move_xp_, 1, 2);
  l_ee->addWidget(btn_move_xm_, 3, 2);
  l_ee->addWidget(btn_move_yp_, 2, 1);
  l_ee->addWidget(btn_move_ym_, 2, 3);
  l_ee->addLayout(l_ee_z, 2, 2);
  layout->addWidget(lb_ee_);
  layout->addLayout(l_ee);
  
  lb_gripper_ = new QLabel(QString::fromStdString("Gripper Action"));
  lb_gripper_->setAlignment(Qt::AlignCenter);
  QHBoxLayout* l_gripper = new QHBoxLayout;  
  l_gripper->addWidget(btn_gripper_open_);
  l_gripper->addWidget(btn_gripper_close_);
  layout->addWidget(lb_gripper_);
  layout->addLayout(l_gripper);

  setLayout(layout);


  //////////////////////////////
  // Initial Setting
  //////////////////////////////

  rbtn_motion_1_->setChecked(true);
  rbtn_appr_3_->setChecked(true);
  set_all_enabled();

  // goal_reached_flag = 0;
  mb_last = 0;
  m_motion = 0; // 0-pick up, 1-place

  mb_status_subscriber_ = nh_.subscribe<std_msgs::Int32>("/cb_gui_mb_status", 1, &CBGuiRviz::mb_status_cb, this);
  cvrg_pcl_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>("/approximate_bb/cloud_trimmed", 100, &CBGuiRviz::cvrg_pcl_cb, this);

}


void CBGuiRviz::mb_status_cb(const std_msgs::Int32::ConstPtr& msg)
{
  int mb_status = msg->data;
  printf("move base status is %d\n", mb_status);

  // message when the goal is reached
  if (mb_status == 2)
  {
    QMessageBox::information(this, "MoveBase", "Plan Canceled by User");
  }
  else if (mb_status == 3)
  {
    QMessageBox::information(this, "MoveBase", "Goal Reached");
  }
  else if (mb_status == 4)
  {
    QMessageBox::information(this, "MoveBase", "Plan Canceled by Oscillation");
  }
}

void CBGuiRviz::cvrg_pcl_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO("Point cloud obtained.");
  pcl::fromROSMsg(*msg, received_cloud_ptr);
  cloud_status = true;
}

void CBGuiRviz::cancelMB()
{
  remote_reciever_.publishCancelMB();
}

void CBGuiRviz::pauseRtabmap()
{
  remote_reciever_.callPauseRtabmap();
}

void CBGuiRviz::resumeRtabmap()
{
  remote_reciever_.callResumeRtabmap();
}

void CBGuiRviz::moveGripperOpen()
{
  remote_reciever_.publishGripperOpen();
}

void CBGuiRviz::moveGripperClose()
{
  remote_reciever_.publishGripperClose();
}

void CBGuiRviz::pclCapture()
{
  std_srvs::Trigger srv;
  if(pcl_capture_client_.call(srv))
    ROS_INFO_STREAM_NAMED("gui", "pcl capture success");
  else
    ROS_ERROR_STREAM_NAMED("gui", "pcl capture failed");
   remote_reciever_.publishPclCapture();
}

void CBGuiRviz::pclReset()
{
  std_srvs::Trigger srv;
  if(pcl_reset_client_.call(srv))
    ROS_INFO_STREAM_NAMED("gui", "pcl reset success");
  else
    ROS_ERROR_STREAM_NAMED("gui", "pcl reset failed");
  if(abb_reset_client_.call(srv))
    ROS_INFO_STREAM_NAMED("gui", "abb reset success");
  else
    ROS_ERROR_STREAM_NAMED("gui", "abb reset failed");
  remote_reciever_.publishPclClear();
}

void CBGuiRviz::abbTrim()
{
  std_srvs::Trigger srv;
  if(abb_trim_client_.call(srv))
    ROS_INFO_STREAM_NAMED("gui", "abb trim success");
  else
    ROS_ERROR_STREAM_NAMED("gui", "abb trim failed");
}

void CBGuiRviz::abbReset()
{
  std_srvs::Trigger srv;
  if(abb_reset_client_.call(srv))
    ROS_INFO_STREAM_NAMED("gui", "abb reset success");
  else
    ROS_ERROR_STREAM_NAMED("gui", "abb reset failed");
  remote_reciever_.publishPclCapture();
}

void CBGuiRviz::abbSave()
{
  std_srvs::Trigger srv;
  if(abb_save_client_.call(srv))
    ROS_INFO_STREAM_NAMED("gui", "abb save success");
  else
    ROS_ERROR_STREAM_NAMED("gui", "abb save failed");
}

int CBGuiRviz::cvrgPlan()
{
  pc_to_mesh::pc_to_mesh pc_to_mesh_srv;
  cvrg_path::cvrg_plan cvrg_srv;


  if(!cloud_status)
  {
    printf("No subscribed pointcloud data");
    return 0;
  }
  else
  {
    sensor_msgs::PointCloud2 object_msg;
    pcl::toROSMsg(received_cloud_ptr, object_msg );
    object_msg.header.frame_id = "/world";

    pc_to_mesh_srv.request.pt_cloud = object_msg;
    if(pc_to_mesh_client_.call(pc_to_mesh_srv)){ //Call the service
      ROS_INFO("pc_to_mesh Service Called Successfully.");
    }
    else{
      ROS_ERROR("pc_to_mesh Service Call Failed.");
      return 0;
    }

    std::vector<double> pose(7);
    ros::param::get("/cvrg_tf_param/pc_pose",pose);

    geometry_msgs::Pose stl_pose;
    stl_pose.position.x = pose[0];
    stl_pose.position.y = pose[1];
    stl_pose.position.z = pose[2];
    stl_pose.orientation.x = pose[3];
    stl_pose.orientation.y = pose[4];
    stl_pose.orientation.z = pose[5];
    stl_pose.orientation.w = pose[6];

    std_msgs::String stl_path;
    ros::param::get("/file_paths/stl_path",stl_path.data);


    cvrg_srv.request.stl_path = stl_path;
    cvrg_srv.request.stl_pose = stl_pose;

    ROS_INFO("Calling Service");
    if(cvrg_client_.call(cvrg_srv)){ //Call the service
        ROS_INFO("cvrg plan Service Called Successfully.");
    }
    else{
        ROS_ERROR("cvrg plan Service Call Failed.");
        return 0;
    }

    ros::param::set("/status/pc_to_mesh",true); // Visualization node checks this status and know that the service call has finished

    return 1;
  }
}

void CBGuiRviz::scanPlan()
{
  std_srvs::Trigger srv;

  // btn_scanning_plan_->setEnabled(false);

  if(scan_plan_client_.call(srv))
    ROS_INFO_STREAM_NAMED("gui", "scan plan success");
  else
    ROS_ERROR_STREAM_NAMED("gui", "scan plan failed");

  // btn_scanning_plan_->setEnabled(true);

}

void CBGuiRviz::scanExecute()
{
  std_srvs::Trigger srv;

  btn_scanning_execute_->setEnabled(false);

  if(scan_execute_client_.call(srv))
    ROS_INFO_STREAM_NAMED("gui", "scanning execute success");
  else
    ROS_ERROR_STREAM_NAMED("gui", "scanning execute failed");

  btn_scanning_execute_->setEnabled(true);
}

void CBGuiRviz::apprPlan()
{
  if(m_motion == 0) remote_reciever_.publishPickApproachPlan();
  else              remote_reciever_.publishPlaceApproachPlan();
}

void CBGuiRviz::apprExecute()
{
  if(m_motion == 0) remote_reciever_.publishPickApproachExecute();
  else              remote_reciever_.publishPlaceApproachExecute();
}

void CBGuiRviz::retrPlan()
{
  if(m_motion == 0) remote_reciever_.publishPickRetreatPlan();
  else              remote_reciever_.publishPlaceRetreatPlan();
}

void CBGuiRviz::retrExecute()
{
  if(m_motion == 0) remote_reciever_.publishPickRetreatExecute();
  else              remote_reciever_.publishPlaceRetreatExecute();
}

void CBGuiRviz::moveXP()
{
  remote_reciever_.publishMoveXP();
}

void CBGuiRviz::moveXM()
{
  remote_reciever_.publishMoveXM();
}

void CBGuiRviz::moveYP()
{
  remote_reciever_.publishMoveYP();
}

void CBGuiRviz::moveYM()
{
  remote_reciever_.publishMoveYM();
}

void CBGuiRviz::moveZP()
{
  remote_reciever_.publishMoveZP();
}

void CBGuiRviz::moveZM()
{
  remote_reciever_.publishMoveZM();
}

void CBGuiRviz::apprRB1()
{
  remote_reciever_.publishRB(1);
}

void CBGuiRviz::apprRB2()
{
  remote_reciever_.publishRB(2);
}

void CBGuiRviz::apprRB3()
{
  remote_reciever_.publishRB(3);
}

void CBGuiRviz::motionRB1()
{
  m_motion = 0;
}

void CBGuiRviz::motionRB2()
{
  m_motion = 1;
}

void CBGuiRviz::setDistance(int value)
{
  remote_reciever_.publishDistance(value);
}

void CBGuiRviz::set_all_disabled()
{
  lb_mb_->setStyleSheet("background-color:rgb(186, 189, 182); color:gray;");
  lb_mb_loc_->setStyleSheet("color:gray;");
  btn_mb_cancel_->setEnabled(false);
  btn_rtabmap_pause_->setEnabled(false);
  btn_rtabmap_resume_->setEnabled(false);  

  lb_pcl_->setStyleSheet("background-color:rgb(186, 189, 182); color:gray;");
  btn_pcl_capture_->setEnabled(false);
  btn_pcl_reset_->setEnabled(false);
  btn_abb_trim_->setEnabled(false);
  btn_abb_reset_->setEnabled(false);
  btn_abb_save_->setEnabled(false);

  lb_appr_->setStyleSheet("background-color:rgb(186, 189, 182); color:gray;");
  lb_appr_pose_->setStyleSheet("color:gray;");
  lb_appr_dist_->setStyleSheet("color:gray;");
  lb_appr_ik_->setStyleSheet("color:gray;");
  rbtn_appr_1_->setEnabled(false);
  rbtn_appr_2_->setEnabled(false);
  rbtn_appr_3_->setEnabled(false);
  slider_->setEnabled(false);
  btn_imarker_clear_->setEnabled(false);
  combo_box_->setEnabled(false);  

  lb_motion_->setStyleSheet("background-color:rgb(186, 189, 182); color:gray;");
  lb_motion_appr_->setStyleSheet("color:gray;");
  lb_motion_retr_->setStyleSheet("color:gray;");
  rbtn_motion_1_->setEnabled(false);
  rbtn_motion_2_->setEnabled(false);
  btn_appr_plan_->setEnabled(false);
  btn_appr_execute_->setEnabled(false);
  btn_retr_plan_->setEnabled(false);
  btn_retr_execute_->setEnabled(false);

  lb_ee_->setStyleSheet("background-color:rgb(186, 189, 182); color:gray;");
  btn_move_xp_->setEnabled(false);
  btn_move_xm_->setEnabled(false);
  btn_move_yp_->setEnabled(false);
  btn_move_ym_->setEnabled(false);
  btn_move_zp_->setEnabled(false);
  btn_move_zm_->setEnabled(false);

  lb_gripper_->setStyleSheet("background-color:rgb(186, 189, 182); color:gray;");
  btn_gripper_open_->setEnabled(false);
  btn_gripper_close_->setEnabled(false);
}

void CBGuiRviz::set_all_enabled()
{
  lb_mb_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
  lb_mb_loc_->setStyleSheet("color:black;");
  btn_mb_cancel_->setEnabled(true);
  btn_rtabmap_pause_->setEnabled(true);
  btn_rtabmap_resume_->setEnabled(true);  

  lb_pcl_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
  btn_pcl_capture_->setEnabled(true);
  btn_pcl_reset_->setEnabled(true);
  btn_abb_trim_->setEnabled(true);
  btn_abb_reset_->setEnabled(true);

  lb_appr_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
  lb_appr_pose_->setStyleSheet("color:black;");
  lb_appr_dist_->setStyleSheet("color:black;");
  lb_appr_ik_->setStyleSheet("color:black;");
  rbtn_appr_1_->setEnabled(true);
  rbtn_appr_2_->setEnabled(true);
  rbtn_appr_3_->setEnabled(true);
  slider_->setEnabled(true);
  btn_imarker_clear_->setEnabled(true);
  combo_box_->setEnabled(true);  

  lb_motion_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
  lb_motion_appr_->setStyleSheet("color:black;");
  lb_motion_retr_->setStyleSheet("color:black;");
  rbtn_motion_1_->setEnabled(true);
  rbtn_motion_2_->setEnabled(true);
  btn_appr_plan_->setEnabled(true);
  btn_appr_execute_->setEnabled(true);
  btn_retr_plan_->setEnabled(true);
  btn_retr_execute_->setEnabled(true);

  lb_ee_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
  btn_move_xp_->setEnabled(true);
  btn_move_xm_->setEnabled(true);
  btn_move_yp_->setEnabled(true);
  btn_move_ym_->setEnabled(true);
  btn_move_zp_->setEnabled(true);
  btn_move_zm_->setEnabled(true);

  lb_gripper_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
  btn_gripper_open_->setEnabled(true);
  btn_gripper_close_->setEnabled(true);
}

void CBGuiRviz::updateText()
{
  text_browser_->setText(QString::fromStdString(remote_reciever_.get_instruction()));
}


void CBGuiRviz::updateMBStatus()
{
  int mb_status = remote_reciever_.get_mb_status();
  printf("move base status is %d\n", mb_status);

  // message when the goal is reached
  if (mb_status == 2)
  {
    QMessageBox::information(this, "MoveBase", "Plan Canceled by User");
  }
  else if (mb_status == 3)
  {
    QMessageBox::information(this, "MoveBase", "Goal Reached");
  }
  else if (mb_status == 4)
  {
    QMessageBox::information(this, "MoveBase", "Plan Canceled by Oscillation");
  }
}

void CBGuiRviz::clearIMarker()
{
  rbtn_appr_3_->setChecked(true);
  slider_->setValue(10);
  remote_reciever_.publishClearIMarker();
}

void CBGuiRviz::selectSolution(int value)
{
  remote_reciever_.publishSolution(value-1);
}

void CBGuiRviz::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void CBGuiRviz::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}
}  // end namespace cb_gui_rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cb_gui_rviz::CBGuiRviz, rviz::Panel)
