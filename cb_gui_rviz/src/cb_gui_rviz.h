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

#ifndef CB_GUI_RVIZ__MM_GUI_RVIZ_H
#define CB_GUI_RVIZ__MM_GUI_RVIZ_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <pc_to_mesh/pc_to_mesh.h>
#include <cvrg_path/cvrg_plan.h>

#include <rviz/panel.h>
#endif

#include <QPushButton>
#include <QComboBox>

#include <QRadioButton>
#include <QTextBrowser>
#include <QSlider>
#include <QTableWidget>
#include <QStringList>
#include <QHeaderView>
#include <QLabel>

#include <cb_gui_rviz/remote_reciever.h>

#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>

class QLineEdit;
class QSpinBox;
class QVBoxLayout;
class QHBoxLayout;
class QFrame;

namespace cb_gui_rviz
{
class CBGuiRviz : public rviz::Panel
{
  Q_OBJECT
public:
  explicit CBGuiRviz(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;
  void cvrg_pcl_cb(const sensor_msgs::PointCloud2ConstPtr&);
  void mb_status_cb(const std_msgs::Int32::ConstPtr&);
public Q_SLOTS:
  
  // void updateText();

  void updateMBStatus();

protected Q_SLOTS:

  void cancelMB();

  void pauseRtabmap();

  void resumeRtabmap();

  void apprPlan();

  void apprExecute();

  void retrPlan();

  void retrExecute();
  
  void moveGripperOpen();
  
  void moveGripperClose();

  void pclCapture();

  void pclReset();

  void abbTrim();

  void abbReset();

  // void abbSave();

  int cvrgPlan();

  void graspWand();

  void putWand();

  void scanPlan();

  void scanExecute();

  void emergencyStop();

  void moveXP();

  void moveXM();

  void moveYP();

  void moveYM();

  void moveZP();

  void moveZM();

  void moveJ1M();

  void moveJ1P();

  void moveJ4M();

  void moveJ4P();

  void moveJ6M();

  void moveJ6P();

  void apprRB1();

  void apprRB2();

  void apprRB3();

  void motionRB1();

  void motionRB2();

  void setDistance(int);

  void clearIMarker();

  void selectSolution(int);

  void set_all_disabled();

  void set_all_enabled();

protected:
  QVBoxLayout* layout;

  QPushButton* btn_mb_cancel_;

  QPushButton* btn_rtabmap_pause_;
  QPushButton* btn_rtabmap_resume_;
  
  QPushButton* btn_gripper_open_;
  QPushButton* btn_gripper_close_;
  
  QPushButton* btn_pcl_capture_;
  QPushButton* btn_pcl_reset_;
  QPushButton* btn_abb_trim_;
  QPushButton* btn_abb_reset_;
  // QPushButton* btn_abb_save_;

  QPushButton* btn_cvrg_plan_;

  QPushButton* btn_grasp_wand_;
  QPushButton* btn_put_wand_;

  QPushButton* btn_scanning_plan_;
  QPushButton* btn_scanning_execute_;

  QPushButton* btn_imarker_clear_;

  QPushButton* btn_appr_plan_;
  QPushButton* btn_appr_execute_;
  QPushButton* btn_retr_plan_;
  QPushButton* btn_retr_execute_;

  QPushButton* btn_move_xp_;
  QPushButton* btn_move_xm_;
  QPushButton* btn_move_yp_;
  QPushButton* btn_move_ym_;
  QPushButton* btn_move_zp_;
  QPushButton* btn_move_zm_;

  QPushButton* btn_move_j1m_;
  QPushButton* btn_move_j1p_;
  QPushButton* btn_move_j4m_;
  QPushButton* btn_move_j4p_;
  QPushButton* btn_move_j6m_;
  QPushButton* btn_move_j6p_;

  QRadioButton* rbtn_appr_1_;
  QRadioButton* rbtn_appr_2_;
  QRadioButton* rbtn_appr_3_;

  QRadioButton* rbtn_motion_1_;
  QRadioButton* rbtn_motion_2_;

  QSlider* slider_;

  QTimer* timer_;
  // QTextBrowser* text_browser_;

  QComboBox *combo_box_;

  QLabel* lb_mb_;
  QLabel* lb_mb_loc_;
  QLabel* lb_pcl_;
  QLabel* lb_san_;
  QLabel* lb_appr_;
  QLabel* lb_appr_pose_;
  QLabel* lb_appr_dist_;
  QLabel* lb_appr_ik_;
  QLabel* lb_motion_;
  QLabel* lb_motion_appr_;
  QLabel* lb_motion_retr_;
  QLabel* lb_ee_;
  QLabel* lb_gripper_;
  
  RemoteReciever remote_reciever_;

  ros::NodeHandle nh_;

  // Publisher
  ros::Publisher scan_plan_publisher_ = nh_.advertise<std_msgs::Bool>("/cb_gui_moveit/scanning_plan", 1);
  ros::Publisher scan_execute_publisher_ = nh_.advertise<std_msgs::Bool>("/cb_gui_moveit/scanning_execute", 1);

  ros::Publisher grasp_wand_publisher_ = nh_.advertise<std_msgs::Bool>("/cb_gui_moveit/grasp_wand", 1);
  ros::Publisher grasp_object_publisher_ = nh_.advertise<std_msgs::Bool>("/cb_gui_moveit/grasp_object", 1);

  ros::Publisher pcl_capture_publisher_ = nh_.advertise<std_msgs::Bool>("/cb_gui_moveit/pcl_capture", 1);
  ros::Publisher pcl_clear_publisher_ = nh_.advertise<std_msgs::Bool>("/cb_gui_moveit/pcl_clear", 1);

  ros::Publisher j1_command_publisher_ = nh_.advertise<std_msgs::Bool>("/cb_gui_moveit/j1", 1);
  ros::Publisher j4_command_publisher_ = nh_.advertise<std_msgs::Bool>("/cb_gui_moveit/j4", 1);
  ros::Publisher j6_command_publisher_ = nh_.advertise<std_msgs::Bool>("/cb_gui_moveit/j6", 1);

  // Subscriber
  ros::Subscriber mb_status_subscriber_;
  ros::Subscriber cvrg_pcl_subscriber_;

  // Service
  ros::ServiceClient pcl_capture_client_ = nh_.serviceClient<std_srvs::Trigger>("/pcl_fusion_node/capture_point_cloud");
  ros::ServiceClient pcl_reset_client_ = nh_.serviceClient<std_srvs::Trigger>("/pcl_fusion_node/reset");

  ros::ServiceClient abb_trim_client_ = nh_.serviceClient<std_srvs::Trigger>("/approximate_bb/trim_point_cloud");
  ros::ServiceClient abb_reset_client_ = nh_.serviceClient<std_srvs::Trigger>("/approximate_bb/reset");
  ros::ServiceClient abb_save_client_ = nh_.serviceClient<std_srvs::Trigger>("/approximate_bb/savePointCloud");

  ros::ServiceClient pc_to_mesh_client_ = nh_.serviceClient<pc_to_mesh::pc_to_mesh>("/greedy_projection");
  ros::ServiceClient cvrg_client_ = nh_.serviceClient<cvrg_path::cvrg_plan>("/gen_cvrg_plan");

  ros::ServiceClient scan_plan_client_ = nh_.serviceClient<std_srvs::Trigger>("/cb_gui_moveit/scanning_plan");
  ros::ServiceClient scan_execute_client_ = nh_.serviceClient<std_srvs::Trigger>("/cb_gui_moveit/scanning_execute");

  // pcl
  pcl::PointCloud<pcl::PointXYZ> received_cloud_ptr;  
  bool cloud_status = false;

  // int goal_reached_flag; 
  int mb_last;
  int m_motion;

};

}  // end namespace cb_gui_rviz

#endif  // CB_GUI_RVIZ__CB_GUI_RVIZ_H