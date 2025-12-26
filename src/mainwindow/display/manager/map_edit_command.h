#pragma once
#include <QImage>
#include <QRectF>
#include <QPointF>
#include <QString>
#include <string>
#include <vector>
#include "map/topology_map.h"

namespace Display {
class SceneManager;
class DisplayOccMap;
class PointShape;

class MapEditCommand {
 public:
  virtual ~MapEditCommand() = default;
  virtual void Undo(SceneManager* manager) = 0;
  virtual void Redo(SceneManager* manager) = 0;
};

class EraseCommand : public MapEditCommand {
 private:
  QRectF region_;
  QImage saved_image_;
  DisplayOccMap* map_ptr_;

 public:
  // 从单个点创建命令（向后兼容）
  EraseCommand(DisplayOccMap* map_ptr, const QPointF& pose, double range);
  // 从区域和保存的图像创建命令（用于连续操作）
  EraseCommand(DisplayOccMap* map_ptr, const QRectF& region, const QImage& saved_image);
  void Undo(SceneManager* manager) override;
  void Redo(SceneManager* manager) override;
};

class DrawPointCommand : public MapEditCommand {
 private:
  QPointF point_;
  QRectF region_;
  QImage saved_image_;
  DisplayOccMap* map_ptr_;
  bool is_continuous_operation_{false};  // 是否为连续操作

 public:
  // 从单个点创建命令（向后兼容）
  DrawPointCommand(DisplayOccMap* map_ptr, const QPointF& point);
  // 从区域和保存的图像创建命令（用于连续操作）
  DrawPointCommand(DisplayOccMap* map_ptr, const QRectF& region, const QImage& saved_image);
  void Undo(SceneManager* manager) override;
  void Redo(SceneManager* manager) override;
};

class DrawLineCommand : public MapEditCommand {
 private:
  QPointF start_;
  QPointF end_;
  QRectF region_;
  QImage saved_image_;
  DisplayOccMap* map_ptr_;

 public:
  // 从起点和终点创建命令（向后兼容）
  DrawLineCommand(DisplayOccMap* map_ptr, const QPointF& start, const QPointF& end);
  // 从起点、终点、区域和保存的图像创建命令（用于连续操作）
  DrawLineCommand(DisplayOccMap* map_ptr, const QPointF& start, const QPointF& end, 
                  const QRectF& region, const QImage& saved_image);
  void Undo(SceneManager* manager) override;
  void Redo(SceneManager* manager) override;
};

class AddPointCommand : public MapEditCommand {
 private:
  std::string point_name_;
  TopologyMap::PointInfo point_info_;

 public:
  AddPointCommand(const std::string& name, const TopologyMap::PointInfo& info);
  void Undo(SceneManager* manager) override;
  void Redo(SceneManager* manager) override;
};

class RemovePointCommand : public MapEditCommand {
 private:
  std::string point_name_;
  TopologyMap::PointInfo point_info_;
  std::vector<std::string> related_routes_;

 public:
  RemovePointCommand(const std::string& name, const TopologyMap::PointInfo& info,
                     const std::vector<std::string>& routes);
  void Undo(SceneManager* manager) override;
  void Redo(SceneManager* manager) override;
};

class AddTopologyLineCommand : public MapEditCommand {
 private:
  QString from_;
  QString to_;
  std::string route_id_;

 public:
  AddTopologyLineCommand(const QString& from, const QString& to);
  void Undo(SceneManager* manager) override;
  void Redo(SceneManager* manager) override;
};

class RemoveTopologyLineCommand : public MapEditCommand {
 private:
  QString from_;
  QString to_;
  std::string route_id_;

 public:
  RemoveTopologyLineCommand(const QString& from, const QString& to);
  void Undo(SceneManager* manager) override;
  void Redo(SceneManager* manager) override;
};

class UpdatePointCommand : public MapEditCommand {
 private:
  std::string point_name_;
  TopologyMap::PointInfo old_point_info_;
  TopologyMap::PointInfo new_point_info_;

 public:
  UpdatePointCommand(const std::string& name, const TopologyMap::PointInfo& old_info, const TopologyMap::PointInfo& new_info);
  void Undo(SceneManager* manager) override;
  void Redo(SceneManager* manager) override;
};

class UpdatePointNameCommand : public MapEditCommand {
 private:
  std::string old_name_;
  std::string new_name_;

 public:
  UpdatePointNameCommand(const std::string& old_name, const std::string& new_name);
  void Undo(SceneManager* manager) override;
  void Redo(SceneManager* manager) override;
};

}  // namespace Display

