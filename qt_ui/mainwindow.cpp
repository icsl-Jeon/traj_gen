#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPixmap>
#include <QSettings>

const double SLIDER_MAX = 5;

MainWindow::MainWindow(QNode* qnode,QWidget *parent) :
    QMainWindow(parent),qnode(qnode),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    // intial configuration

    // logos
    string cd = __FILE__;
    cd.erase(cd.end()-14,cd.end());    
    QPixmap pix_larr((cd+"/resources/LARR.jpg").c_str());
    int w = ui->label_larr->width();
    int h = ui->label_larr->height();
    ui->label_larr->setPixmap(pix_larr.scaled(w,h,Qt::KeepAspectRatio));

    QPixmap pix_larr2((cd + "/resources/maxresdefault.jpg").c_str());
    int w2 = ui->label_larr_2->width();
    int h2 = ui->label_larr_2->height();
    ui->label_larr_2->setPixmap(pix_larr2.scaled(w2,h2,Qt::KeepAspectRatio));

    

    // push button color
    ui->pushButton_ros->setStyleSheet("  QPushButton:checked{background-color: rgba(200, 20, 80,20);\
                                          }");

    ui->pushButton_waypoint->setStyleSheet("QPushButton:checked{background-color: rgba(0, 100, 80,20);\
                                           }");

    ui->pushButton_publish->setStyleSheet("QPushButton:checked{background-color: rgba(0, 20, 100,50);\
                                           }");

    QObject::connect(qnode,SIGNAL(writeOnBoard(QString)),this,SLOT(textEdit_write(QString)));
    QObject::connect(qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(qnode, SIGNAL(askSlider(double *)), this, SLOT(get_slider_value(double *)));   
    ReadSettings();

    ui->lineEdit_n_corridor->setDisabled(true);
    ui->lineEdit_deviation_weight->setDisabled(true);

    std::ostringstream strs1;
    strs1 << SLIDER_MAX;
    std::string str1 = strs1.str();
    
    std::ostringstream strs2;
    strs2 << SLIDER_MAX/2.0;
    std::string str2 = strs2.str();
    
    ui->label_slider_upper->setText(str1.c_str());
    ui->label_slider_middle->setText(str2.c_str());

}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event){

        qnode->shutdown();
        WriteSettings();
        QMainWindow::closeEvent(event);
}


void MainWindow::ReadSettings(){
    QSettings settings("path_planner", qnode->nodeName().c_str());

    // setting names    
    QString filename = settings.value("filename",QString("path_saved.txt")).toString();
    QString poly_order = settings.value("poly_order",QString("6")).toString();
    QString simulation_tf = settings.value("tf", QString("20")).toString();
    QString weight_on_deviation = settings.value("w_d", QString("2")).toString();
    QString safe_rad = settings.value("safe_radius",QString("0.5")).toString();
    QString num_per_seg = settings.value("number_per_seg",QString("2")).toString();
    QString derivative = settings.value("derivative",QString("3")).toString();
 
    
    // fill with previous settings 
    ui->lineEdit_load_directory->setText(filename);
    ui->lineEdit_sim_tf->setText(simulation_tf);
    ui->lineEdit_deviation_weight->setText(weight_on_deviation);
    ui->lineEdit_n_corridor->setText(num_per_seg);
    ui->lineEdit_safe_radius->setText(safe_rad);
    ui->lineEdit_poly_order->setText(poly_order);
    ui->lineEdit_derivative->setText(derivative);

    
}


void MainWindow::WriteSettings(){

    QSettings settings("path_planner", qnode->nodeName().c_str());
    
    settings.setValue("geometry", geometry());
    settings.setValue("windowState", saveState());

    settings.setValue("filename",ui->lineEdit_load_directory->text());
    settings.setValue("tf",ui->lineEdit_sim_tf->text());
    settings.setValue("poly_order",ui->lineEdit_poly_order->text());
    settings.setValue("w_d",ui->lineEdit_deviation_weight->text());
    settings.setValue("safe_radius",ui->lineEdit_safe_radius->text());
    settings.setValue("number_per_seg",ui->lineEdit_n_corridor->text());
    settings.setValue("derivative",ui->lineEdit_derivative->text());

}


void MainWindow::on_pushButton_clear_clicked(){
    // initialize the queue in qnode 
    qnode->queue.clear();
    qnode->wpnt_markerArray.markers.clear();

}

void MainWindow::on_pushButton_undo_clicked(){
    // remove the back element
    qnode->queue.pop_back();
    qnode->wpnt_markerArray.markers.pop_back();
}
void MainWindow::on_pushButton_ros_clicked(bool checked)
{

    if(qnode->on_init()){
        ui->textEdit_message->append("ros connected.");
        qnode->is_connected = true;

    }else{
        ui->textEdit_message->append("failed. retry");
    }


}

void MainWindow::on_pushButton_save_clicked(){


    // file write
    std::ofstream wnpt_file;
    string filename = ui->lineEdit_load_directory->text().toStdString();
    wnpt_file.open(filename);

    if(wnpt_file.is_open()){
        for(auto it = qnode->queue.begin();it<qnode->queue.end();it++){
            wnpt_file<<std::to_string(it->pose.position.x)<<","<<std::to_string(it->pose.position.y)<<","<<std::to_string(it->pose.position.z)<<"\n";
        }
        wnpt_file.close();
        ui->textEdit_message->append(QString((string("to ") + filename + string(", written")).data()));

    }else
        ui->textEdit_message->append(QString("file not written."));

}

void MainWindow::on_pushButton_waypoint_clicked(bool checked){
    if(ui->pushButton_waypoint->isChecked()){
        
        ui->textEdit_message->append("please select waypoints");
        qnode->is_insert_permit = true;

    }else{

        ui->textEdit_message->append("finishing waypoints selection");
        qnode->is_insert_permit = false;
    }

}

void MainWindow::on_pushButton_trajectory_clicked()
{
    // path generation requested
    if (qnode->queue.size()){    
    //parameter parsing

    double tf = atoi(ui->lineEdit_sim_tf->text().toStdString().c_str());

    TrajGenOpts option;
    option.poly_order = atoi(ui->lineEdit_poly_order->text().toStdString().c_str());
    option.objective_derivative = atoi(ui->lineEdit_derivative->text().toStdString().c_str());
    option.is_waypoint_soft = ui->checkBox_is_soft->isChecked();
    option.is_single_corridor = ui->checkBox_is_single->isChecked();
    option.is_multi_corridor = ui->checkBox_is_multi->isChecked();
    option.w_d = atoi(ui->lineEdit_deviation_weight->text().toStdString().c_str());
    option.N_safe_pnts = atoi(ui->lineEdit_n_corridor->text().toStdString().c_str());
    option.safe_r = atof(ui->lineEdit_safe_radius->text().toStdString().c_str());
    
    if(qnode->traj_gen_call(tf,geometry_msgs::Twist(),geometry_msgs::Twist(),option))
        ui->textEdit_message->append("trajectory obtained");
    else
        ui->textEdit_message->append("generation failed.");
    }
    else
        ui->textEdit_message->append("No waypoints yet.");
}

void MainWindow::on_checkBox_is_multi_clicked(){

    if(ui->checkBox_is_multi->isChecked()){
        ui->lineEdit_n_corridor->setEnabled(true);
        if(ui->checkBox_is_single->isChecked())
            ui->checkBox_is_single->setChecked(false);
    }else{

        ui->lineEdit_n_corridor->setDisabled(true);
    }
}

void MainWindow::on_checkBox_is_soft_clicked(){

    if(ui->checkBox_is_soft->isChecked()){
        ui->lineEdit_deviation_weight->setEnabled(true);
    }else{
        ui->lineEdit_deviation_weight->setDisabled(true);
    }

}
// single box safety 
void MainWindow::on_checkBox_is_single_clicked(){

     if(ui->checkBox_is_single->isChecked()){
        if(ui->checkBox_is_multi->isChecked()){
            ui->checkBox_is_multi->setChecked(false);
            ui->lineEdit_n_corridor->setDisabled(true);
        }
    }
}

void MainWindow::on_pushButton_load_clicked()
{
    // file read : queue wil be filled with these

    string filename = ui->lineEdit_load_directory->text().toStdString();
    std::ifstream infile;
    infile.open(filename);
    if(infile.is_open())
        ui->textEdit_message->append(QString("pnts reading.."));
    else
    {
        ui->textEdit_message->append(QString("could not open file."));
        return;
    }

    std::vector<geometry_msgs::PoseStamped> queue_replace;

    while (! infile.eof()){
        std::string line;
        getline(infile, line); // if no delimiter given, new line is that
        // std::cout<<line<<std::endl;
        std::stringstream stream(line);
        std::string val;
        int xyz_idx = 0;
        geometry_msgs::PoseStamped wpnt;

        while(! stream.eof()) {
            getline(stream, val, ',');
            if(xyz_idx == 0)
                wpnt.pose.position.x = atof(val.c_str());
            else if(xyz_idx == 1)
                wpnt.pose.position.y = atof(val.c_str());
            else
                wpnt.pose.position.z = atof(val.c_str());
            xyz_idx ++;
        }
        queue_replace.push_back(wpnt);
        // std::cout<< wpnt.pose.position.x <<" , "<< wpnt.pose.position.y <<" , "<<wpnt.pose.position.z<<std::endl;
    }

    queue_replace.pop_back();
    qnode->queue_file_load(0,queue_replace);
}

void MainWindow::on_pushButton_publish_clicked()
{
    if(ui->pushButton_publish->isChecked()){        
        ui->textEdit_message->append("Move control pose from current pose");
        qnode->is_in_session = true;
        qnode->button_click_time = ros::Time::now();
    
    }else{
        ui->textEdit_message->append("stop publishing.");
        qnode->is_in_session = false;
        qnode->previous_elapsed = (ros::Time::now() - qnode->button_click_time).toSec() + qnode->previous_elapsed; // total elasped time
    }

}

void MainWindow::textEdit_write(QString line){    
    ui->textEdit_message->append(line);
};

void MainWindow::get_slider_value(double * height){
    *height = double(ui->verticalSlider->sliderPosition())/double(ui->verticalSlider->maximum()) * SLIDER_MAX; 
    std::cout<<"slider: "<<ui->verticalSlider->sliderPosition()<<std::endl;
           
} 