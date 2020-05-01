#ifndef CB_GUI_RVIZ_POINT_TOOL_H
#define CB_GUI_RVIZ_POINT_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros/node_handle.h>
# include <ros/publisher.h>

# include <rviz/tool.h>

# include <QCursor>
# include <QObject>
#endif

namespace rviz
{
  class StringProperty;
  class BoolProperty;
}

namespace cb_gui_rviz
{
//! The Point Tool allows the user to click on a point which
//! gets published as a PointStamped message.
class PointTool : public rviz::Tool
{
  Q_OBJECT
public:
  PointTool();
  virtual ~PointTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

public Q_SLOTS:

  void updateTopic();
  void updateAutoDeactivate();

protected:
  QCursor std_cursor_;
  QCursor hit_cursor_;

  ros::NodeHandle nh_;
  ros::Publisher pub_;

  // StringProperty* topic_property_;
  rviz::BoolProperty* auto_deactivate_property_;
};

}

#endif