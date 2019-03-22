#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPixmap>
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // intial configuration
    QPixmap pix_larr("/home/jbs/catkin_ws/src/traj_gen/qt_ui/resources/LARR.jpg");
    int w = ui->label_larr->width();
    int h = ui->label_larr->height();
    ui->label_larr->setPixmap(pix_larr.scaled(w,h,Qt::KeepAspectRatio));

    QPixmap pix_larr2("/home/jbs/catkin_ws/src/traj_gen/qt_ui/resources/maxresdefault.jpg");
    int w2 = ui->label_larr_2->width();
    int h2 = ui->label_larr_2->height();
    ui->label_larr_2->setPixmap(pix_larr2.scaled(w2,h2,Qt::KeepAspectRatio));

    ui->pushButton_ros->setStyleSheet("  QPushButton:checked{background-color: rgba(200, 20, 80,20);\
                                          }");

    ui->pushButton_waypoint->setStyleSheet("QPushButton:checked{background-color: rgba(0, 100, 80,20);\
                                           }");

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_ros_clicked(bool checked)
{

}
