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
    QCheckBox *checkBox_2;
    QFrame *line_3;
    QFrame *line_5;
    QCheckBox *checkBox_3;
    QLabel *label_6;
    QLineEdit *lineEdit_safe_radius;
    QFrame *line_6;
    QLabel *label_7;
    QLineEdit *lineEdit_n_corridor;
    QTextEdit *textEdit_message;
    QLabel *label_larr;
    QLabel *label_larr_2;
    QLabel *label_8;
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
        label_2->setGeometry(QRect(10, 100, 101, 17));
        label_2->setFont(font1);
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 180, 67, 17));
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
        lineEdit_deviation_weight->setGeometry(QRect(230, 157, 113, 20));
        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(60, 150, 141, 17));
        line_2 = new QFrame(groupBox);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(-10, 290, 371, 41));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        checkBox_2 = new QCheckBox(groupBox);
        checkBox_2->setObjectName(QString::fromUtf8("checkBox_2"));
        checkBox_2->setGeometry(QRect(30, 130, 141, 22));
        line_3 = new QFrame(groupBox);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(210, 30, 20, 271));
        line_3->setFrameShadow(QFrame::Raised);
        line_3->setLineWidth(1);
        line_3->setFrameShape(QFrame::VLine);
        line_5 = new QFrame(groupBox);
        line_5->setObjectName(QString::fromUtf8("line_5"));
        line_5->setGeometry(QRect(40, 150, 16, 21));
        line_5->setFrameShadow(QFrame::Raised);
        line_5->setFrameShape(QFrame::VLine);
        checkBox_3 = new QCheckBox(groupBox);
        checkBox_3->setObjectName(QString::fromUtf8("checkBox_3"));
        checkBox_3->setGeometry(QRect(30, 230, 171, 22));
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
        textEdit_message = new QTextEdit(centralWidget);
        textEdit_message->setObjectName(QString::fromUtf8("textEdit_message"));
        textEdit_message->setGeometry(QRect(390, 130, 271, 191));
        label_larr = new QLabel(centralWidget);
        label_larr->setObjectName(QString::fromUtf8("label_larr"));
        label_larr->setGeometry(QRect(30, 320, 171, 61));
        label_larr_2 = new QLabel(centralWidget);
        label_larr_2->setObjectName(QString::fromUtf8("label_larr_2"));
        label_larr_2->setGeometry(QRect(540, 330, 121, 61));
        label_8 = new QLabel(centralWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(390, 330, 151, 51));
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
        pushButton_publish->setText(QApplication::translate("MainWindow", "Publish", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindow", "Planning parameters", 0, QApplication::UnicodeUTF8));
        lineEdit_poly_order->setText(QApplication::translate("MainWindow", "6", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Common", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Waypoints", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Safety", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "polynomial order", 0, QApplication::UnicodeUTF8));
        lineEdit_deviation_weight->setText(QApplication::translate("MainWindow", "2", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "weight on deviation", 0, QApplication::UnicodeUTF8));
        checkBox_2->setText(QApplication::translate("MainWindow", "Soft constraint", 0, QApplication::UnicodeUTF8));
        checkBox_3->setText(QApplication::translate("MainWindow", "Multi corridor", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindow", "safe radius", 0, QApplication::UnicodeUTF8));
        lineEdit_safe_radius->setText(QApplication::translate("MainWindow", "1.0", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MainWindow", "number per seg", 0, QApplication::UnicodeUTF8));
        lineEdit_n_corridor->setText(QApplication::translate("MainWindow", "3", 0, QApplication::UnicodeUTF8));
        label_larr->setText(QString());
        label_larr_2->setText(QString());
        label_8->setText(QApplication::translate("MainWindow", "Maintainer: \n"
" Boseong Felipe Jeon \n"
" junbs95@gmail.com  ", 0, QApplication::UnicodeUTF8));
        menuPath_planner->setTitle(QApplication::translate("MainWindow", "path planner", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
