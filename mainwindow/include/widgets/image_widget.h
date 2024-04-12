#pragma once

#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>
#include <mutex>
#include <opencv2/imgproc/imgproc.hpp>
class ImageWidget : public QWidget {
  Q_OBJECT

 public:
  ImageWidget(QWidget* parent = nullptr) {
    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(&imageLabel_);
    this->setLayout(layout);
  }
  ~ImageWidget() {}
  void UpdateImage(std::shared_ptr<cv::Mat> image) {
    QImage img(image->data, image->cols, image->rows, image->step[0], QImage::Format_RGB888);

    // 将 QImage 显示在 QLabel 中
    imageLabel_.setPixmap(QPixmap::fromImage(img));
    imageLabel_.setScaledContents(true);  // 自适应大小显示
  }

 private:
  QLabel imageLabel_;
};