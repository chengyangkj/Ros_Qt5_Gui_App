#pragma once
#include <QGraphicsScene>

class SceneManager : public QGraphicsScene {
public:
  SceneManager(QObject *parent = nullptr) : QGraphicsScene(parent) {}
  virtual ~SceneManager() = default;
};