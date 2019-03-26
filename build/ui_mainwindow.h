/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QFrame>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTextEdit>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *pushButton_ros;
    QPushButton *pushButton_waypoint;
    QPushButton *pushButton_trajectory;
    QPushButton *pushButton_publish;
    QGroupBox *groupBox;
    QLineEdit *lineEdit_poly_order;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QFrame *line;
    QLabel *label_4;
    QLineEdit *lineEdit_deviation_weight;
    QLabel *label_5;
    QFrame *line_2;
    QCheckBox *checkBox_is_soft;
    QFrame *line_3;
    QFrame *line_5;
    QCheckBox *checkBox_is_multi;
    QLabel *label_6;
    QLineEdit *lineEdit_safe_radius;
    QFrame *line_6;
    QLabel *label_7;
    QLineEdit *lineEdit_n_corridor;
    QLabel *label_9;
    QLineEdit *lineEdit_sim_tf;
    QLabel *label_10;
    QLineEdit *lineEdit_derivative;
    QCheckBox *checkBox_is_single;
    QTextEdit *textEdit_message;
    QLabel *label_larr;
    QLabel *label_larr_2;
    QLabel *label_8;
    QPushButton *pushButton_load;
    QLineEdit *lineEdit_load_directory;
    QPushButton *pushButton_save;
    QPushButton *pushButton_undo;
    QPushButton *pushButton_clear;
    QMenuBar *menuBar;
    QMenu *menuPath_planner;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->setEnabled(true);
        MainWindow->resize(669, 448);
        MainWindow->setMouseTracking(true);
        MainWindow->setAutoFillBackground(false);
        MainWindow->setStyleSheet(QString::fromUtf8("backgroundColourButton->setStyleSheet(\"background-color: red\");"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        centralWidget->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        pushButton_ros = new QPushButton(centralWidget);
        pushButton_ros->setObjectName(QString::fromUtf8("pushButton_ros"));
        pushButton_ros->setGeometry(QRect(390, 10, 131, 51));
        pushButton_ros->setAutoFillBackground(false);
        pushButton_ros->setCheckable(true);
        pushButton_ros->setChecked(false);
        pushButton_ros->setAutoDefault(false);
        pushButton_ros->setDefault(false);
        pushButton_ros->setFlat(false);
        pushButton_waypoint = new QPushButton(centralWidget);
        pushButton_waypoint->setObjectName(QString::fromUtf8("pushButton_waypoint"));
        pushButton_waypoint->setGeometry(QRect(530, 10, 131, 51));
        pushButton_waypoint->setCheckable(true);
        pushButton_trajectory = new QPushButton(centralWidget);
        pushButton_trajectory->setObjectName(QString::fromUtf8("pushButton_trajectory"));
        pushButton_trajectory->setGeometry(QRect(390, 70, 131, 51));
        pushButton_publish = new QPushButton(centralWidget);
        pushButton_publish->setObjectName(QString::fromUtf8("pushButton_publish"));
        pushButton_publish->setGeometry(QRect(530, 70, 131, 51));
        pushButton_publish->setCheckable(true);
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(20, 10, 361, 311));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        groupBox->setFont(font);
        groupBox->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 250, 250);"));
        lineEdit_poly_order = new QLineEdit(groupBox);
        lineEdit_poly_order->setObjectName(QString::fromUtf8("lineEdit_poly_order"));
        lineEdit_poly_order->setGeometry(QRect(232, 60, 111, 20));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 40, 91, 16));
        QFont font1;
        font1.setBold(true);
        font1.setItalic(false);
        font1.setWeight(75);
        label->setFont(font1);
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(10, 123, 101, 17));
        label_2->setFont(font1);
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 190, 67, 17));
        label_3->setFont(font1);
        line = new QFrame(groupBox);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(0, 20, 361, 16));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(30, 60, 161, 20));
        lineEdit_deviation_weight = new QLineEdit(groupBox);
        lineEdit_deviation_weight->setObjectName(QString::fromUtf8("lineEdit_deviation_weight"));
        lineEdit_deviation_weight->setGeometry(QRect(230, 170, 113, 20));
        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(60, 163, 141, 17));
        line_2 = new QFrame(groupBox);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(-10, 290, 371, 41));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        checkBox_is_soft = new QCheckBox(groupBox);
        checkBox_is_soft->setObjectName(QString::fromUtf8("checkBox_is_soft"));
        checkBox_is_soft->setGeometry(QRect(30, 143, 141, 22));
        line_3 = new QFrame(groupBox);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(210, 30, 20, 271));
        line_3->setFrameShadow(QFrame::Raised);
        line_3->setLineWidth(1);
        line_3->setFrameShape(QFrame::VLine);
        line_5 = new QFrame(groupBox);
        line_5->setObjectName(QString::fromUtf8("line_5"));
        line_5->setGeometry(QRect(40, 163, 16, 21));
        line_5->setFrameShadow(QFrame::Raised);
        line_5->setFrameShape(QFrame::VLine);
        checkBox_is_multi = new QCheckBox(groupBox);
        checkBox_is_multi->setObjectName(QString::fromUtf8("checkBox_is_multi"));
        checkBox_is_multi->setGeometry(QRect(30, 230, 171, 22));
        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(30, 210, 141, 17));
        lineEdit_safe_radius = new QLineEdit(groupBox);
        lineEdit_safe_radius->setObjectName(QString::fromUtf8("lineEdit_safe_radius"));
        lineEdit_safe_radius->setGeometry(QRect(230, 210, 113, 20));
        line_6 = new QFrame(groupBox);
        line_6->setObjectName(QString::fromUtf8("line_6"));
        line_6->setGeometry(QRect(40, 250, 16, 16));
        line_6->setFrameShadow(QFrame::Raised);
        line_6->setFrameShape(QFrame::VLine);
        label_7 = new QLabel(groupBox);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(60, 250, 121, 17));
        lineEdit_n_corridor = new QLineEdit(groupBox);
        lineEdit_n_corridor->setObjectName(QString::fromUtf8("lineEdit_n_corridor"));
        lineEdit_n_corridor->setGeometry(QRect(230, 250, 113, 20));
        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(30, 80, 121, 17));
        lineEdit_sim_tf = new QLineEdit(groupBox);
        lineEdit_sim_tf->setObjectName(QString::fromUtf8("lineEdit_sim_tf"));
        lineEdit_sim_tf->setGeometry(QRect(232, 80, 111, 20));
        label_10 = new QLabel(groupBox);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(30, 100, 161, 17));
        lineEdit_derivative = new QLineEdit(groupBox);
        lineEdit_derivative->setObjectName(QString::fromUtf8("lineEdit_derivative"));
        lineEdit_derivative->setGeometry(QRect(232, 100, 111, 20));
        checkBox_is_single = new QCheckBox(groupBox);
        checkBox_is_single->setObjectName(QString::fromUtf8("checkBox_is_single"));
        checkBox_is_single->setGeometry(QRect(30, 270, 151, 22));
        textEdit_message = new QTextEdit(centralWidget);
        textEdit_message->setObjectName(QString::fromUtf8("textEdit_message"));
        textEdit_message->setGeometry(QRect(390, 190, 271, 131));
        label_larr = new QLabel(centralWidget);
        label_larr->setObjectName(QString::fromUtf8("label_larr"));
        label_larr->setGeometry(QRect(30, 320, 171, 61));
        label_larr_2 = new QLabel(centralWidget);
        label_larr_2->setObjectName(QString::fromUtf8("label_larr_2"));
        label_larr_2->setGeometry(QRect(540, 330, 121, 61));
        label_8 = new QLabel(centralWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(390, 330, 151, 51));
        pushButton_load = new QPushButton(centralWidget);
        pushButton_load->setObjectName(QString::fromUtf8("pushButton_load"));
        pushButton_load->setGeometry(QRect(460, 160, 61, 20));
        lineEdit_load_directory = new QLineEdit(centralWidget);
        lineEdit_load_directory->setObjectName(QString::fromUtf8("lineEdit_load_directory"));
        lineEdit_load_directory->setGeometry(QRect(390, 130, 271, 21));
        pushButton_save = new QPushButton(centralWidget);
        pushButton_save->setObjectName(QString::fromUtf8("pushButton_save"));
        pushButton_save->setGeometry(QRect(390, 160, 61, 21));
        pushButton_undo = new QPushButton(centralWidget);
        pushButton_undo->setObjectName(QString::fromUtf8("pushButton_undo"));
        pushButton_undo->setGeometry(QRect(530, 160, 61, 21));
        pushButton_clear = new QPushButton(centralWidget);
        pushButton_clear->setObjectName(QString::fromUtf8("pushButton_clear"));
        pushButton_clear->setGeometry(QRect(600, 160, 61, 21));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 669, 25));
        menuPath_planner = new QMenu(menuBar);
        menuPath_planner->setObjectName(QString::fromUtf8("menuPath_planner"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuPath_planner->menuAction());

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        pushButton_ros->setText(QApplication::translate("MainWindow", "ROS connect", 0, QApplication::UnicodeUTF8));
        pushButton_waypoint->setText(QApplication::translate("MainWindow", "Select waypoints", 0, QApplication::UnicodeUTF8));
        pushButton_trajectory->setText(QApplication::translate("MainWindow", "Traj generation", 0, QApplication::UnicodeUTF8));
        pushButton_publish->setText(QApplication::translate("MainWindow", "Publish control", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindow", "Planning parameters", 0, QApplication::UnicodeUTF8));
        lineEdit_poly_order->setText(QApplication::translate("MainWindow", "6", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Common", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Waypoints", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Safety", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "polynomial order", 0, QApplication::UnicodeUTF8));
        lineEdit_deviation_weight->setText(QApplication::translate("MainWindow", "2", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "weight on deviation", 0, QApplication::UnicodeUTF8));
        checkBox_is_soft->setText(QApplication::translate("MainWindow", "Soft constraint", 0, QApplication::UnicodeUTF8));
        checkBox_is_multi->setText(QApplication::translate("MainWindow", "Multi corridor", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindow", "safe radius", 0, QApplication::UnicodeUTF8));
        lineEdit_safe_radius->setText(QApplication::translate("MainWindow", "1.0", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MainWindow", "number per seg", 0, QApplication::UnicodeUTF8));
        lineEdit_n_corridor->setText(QApplication::translate("MainWindow", "3", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("MainWindow", "simulation tf", 0, QApplication::UnicodeUTF8));
        lineEdit_sim_tf->setText(QApplication::translate("MainWindow", "20", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("MainWindow", "objective derivative", 0, QApplication::UnicodeUTF8));
        checkBox_is_single->setText(QApplication::translate("MainWindow", "Single corridor", 0, QApplication::UnicodeUTF8));
        label_larr->setText(QString());
        label_larr_2->setText(QString());
        label_8->setText(QApplication::translate("MainWindow", "Maintainer: \n"
" Boseong Felipe Jeon \n"
" junbs95@gmail.com  ", 0, QApplication::UnicodeUTF8));
        pushButton_load->setText(QApplication::translate("MainWindow", "load", 0, QApplication::UnicodeUTF8));
        pushButton_save->setText(QApplication::translate("MainWindow", "save", 0, QApplication::UnicodeUTF8));
        pushButton_undo->setText(QApplication::translate("MainWindow", "undo", 0, QApplication::UnicodeUTF8));
        pushButton_clear->setText(QApplication::translate("MainWindow", "clear", 0, QApplication::UnicodeUTF8));
        menuPath_planner->setTitle(QApplication::translate("MainWindow", "path planner", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
