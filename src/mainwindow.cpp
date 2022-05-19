#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QLineEdit>
#include <QLabel>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("IEngine - Werasaimon");

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(Update()));

    for(int i=0; i < 6 + 3; ++i)
    {
         ui->widget_Plot->addGraph();
         QPen pen1;
         pen1.setWidth(3);
         pen1.setColor(QColor(rand()%255,rand()%255,rand()%255));
         ui->widget_Plot->graph(i)->setPen(pen1);
    }

    Numer = 0;

    // give the axes some labels:
    ui->widget_Plot->xAxis->setLabel("Time");
    ui->widget_Plot->yAxis->setLabel("Data");
    // set axes ranges, so we see all data:
    ui->widget_Plot->xAxis->setRange(0, MAX);
    ui->widget_Plot->yAxis->setRange(-180, 180);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    ui->widget->keyPressEvent(event);
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    ui->widget->keyReleaseEvent(event);
}


void MainWindow::Update()
{

    const scalar AngleYaw =
    static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDebugAnalisys.AngleYaw * 180.0/M_PI;

    const scalar AngleYaw_Derivative =
    static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDebugAnalisys.AngleYaw_Derivative * 180.0/M_PI;



    const scalar dirivative_motor_a =
    static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDebugAnalisys.dirivative_motor_a * 10;

    const scalar dirivative_motor_b =
    static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDebugAnalisys.dirivative_motor_b * 10;

    const scalar dirivative_motor_c =
    static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDebugAnalisys.dirivative_motor_c * 10;

    const scalar dirivative_motor_d =
    static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDebugAnalisys.dirivative_motor_d * 10;



    const scalar angle_X =
    static_cast<SceneEngineRobocar*>(ui->widget->scene())->m_AngleGimbal.x * 180.0/M_PI;

    const scalar angle_Y  =
    static_cast<SceneEngineRobocar*>(ui->widget->scene())->m_AngleGimbal.y * 180.0/M_PI;

    const scalar angle_Z =
    static_cast<SceneEngineRobocar*>(ui->widget->scene())->m_AngleGimbal.z * 180.0/M_PI;

    if(Numer > MAX - 10)
    {
        /*if(ui->checkBox_AngleYaw->isChecked()) */ ClearData("AngleYaw",0);
        /*if(ui->checkBox_AngleYaw_Derivative->isChecked())*/  ClearData("AngleYaw_Derivative",1);

        /*if(ui->checkBox_motor_A->isChecked())*/ ClearData("motor_A",2);
        /*if(ui->checkBox_motor_B->isChecked())*/ ClearData("motor_B",3);
        /*if(ui->checkBox_motor_C->isChecked())*/ ClearData("motor_C",4);
        /*if(ui->checkBox_motor_D->isChecked())*/ ClearData("motor_D",5);


        ClearData("Angle_Gimbal_X",6);
        ClearData("Angle_Gimbal_Y",7);
        ClearData("Angle_Gimbal_Z",8);


        Numer=0;
    }

    Numer+=0.1;
    N = 0;



    if(ui->checkBox_AngleYaw->isChecked()) PushData("AngleYaw",AngleYaw,0);
    if(ui->checkBox_AngleYaw_Derivative->isChecked()) PushData("AngleYaw_Derivative",AngleYaw_Derivative,1);

    if(ui->checkBox_motor_A->isChecked()) PushData("motor_A",dirivative_motor_a,2);
    if(ui->checkBox_motor_B->isChecked()) PushData("motor_B",dirivative_motor_b,3);
    if(ui->checkBox_motor_C->isChecked()) PushData("motor_C",dirivative_motor_c,4);
    if(ui->checkBox_motor_D->isChecked()) PushData("motor_D",dirivative_motor_d,5);

    if(ui->checkBox_AngleGimbal_X->isChecked()) PushData("Angle_Gimbal_X",angle_X,6);
    if(ui->checkBox_AngleGimbal_Y->isChecked()) PushData("Angle_Gimbal_Y",angle_Y,7);
    //if(ui->checkBox_motor_C->isChecked()) PushData("Angle_Gimbal_Z",angle_Z,8);

    ui->widget_Plot->replot();

    /**
    const Vector3 eulerAngle =
    static_cast<SceneEngineNuzzleGimbal*>(ui->widget->scene())->gimbalStabilization()->mAngleGimbalStabilization;

    if(Numer > MAX - 10)
    {
        _X_EulerX.clear();
        _Y_EulerX.clear();

        _X_EulerY.clear();
        _Y_EulerY.clear();

        _X_EulerZ.clear();
        _Y_EulerZ.clear();

        Numer=0;
    }

     Numer+=0.1;
    _X_EulerX.append(Numer);
    _Y_EulerX.append(eulerAngle.x * 180.0/M_PI);
    ui->widget_Plot->graph(0)->setData(_X_EulerX, _Y_EulerX);
     // ui->widget_Plot->graph(0)->rescaleAxes();

    _X_EulerY.append(Numer);
    _Y_EulerY.append(eulerAngle.y * 180.0/M_PI);
    ui->widget_Plot->graph(1)->setData(_X_EulerY, _Y_EulerY);
     // ui->widget_Plot->graph(1)->rescaleAxes();

    _X_EulerZ.append(Numer);
    _Y_EulerZ.append(eulerAngle.z * 180.0/M_PI);
    ui->widget_Plot->graph(2)->setData(_X_EulerZ, _Y_EulerZ);
     // ui->widget_Plot->graph(2)->rescaleAxes();

    ui->widget_Plot->replot();
    /**/

}

void MainWindow::on_pushButton_Timer_clicked()
{
    if(!timer->isActive())
    {
       timer->start(100);
       m_ISPloter= true;
       ui->pushButton_Timer->setText("Plotter - ON");
    }
    else
    {
       m_ISPloter = false;
       timer->stop();
       ui->pushButton_Timer->setText("Plotter - OFF");
    }
}


void MainWindow::on_checkBox_Range_toggled(bool checked)
{
    static_cast<SceneEngineNuzzleGimbal*>(ui->widget->scene())->gimbalStabilization()->m_isRange = checked;
}


void MainWindow::on_checkBox_Gimbal_toggled(bool checked)
{
   static_cast<SceneEngineNuzzleGimbal*>(ui->widget->scene())->gimbalStabilization()->m_isStabilizationGimbal = checked;
}


void MainWindow::on_pushButton_simulate_clicked()
{
    if(m_IsSimulatePhysics)
    {
       m_IsSimulatePhysics = false;
       ui->pushButton_simulate->setText("Simulate - ON");
    }
    else
    {
       m_IsSimulatePhysics = true;
       ui->pushButton_simulate->setText("Simulate - OFF");
    }

    static_cast<SceneEngineRobocar*>(ui->widget->scene())->mSceneDscriptor.m_IsSimulateDynamics = !m_IsSimulatePhysics;
}


void MainWindow::on_horizontalSlider_Deverative0_sliderMoved(int position)
{
   static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDispatcherAttribute.coffPosDeverative0 = position;
}


void MainWindow::on_horizontalSlider_Deverative1_sliderMoved(int position)
{
   static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDispatcherAttribute.coffPosDeverative1 = position;
}


void MainWindow::on_horizontalSlider_Deverative2_sliderMoved(int position)
{
   static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDispatcherAttribute.coffPosDeverative2 = position;
}


void MainWindow::on_horizontalSlider_RotDev_sliderMoved(int position)
{
   static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDispatcherAttribute.coffRotDev = position / 10.f;
}


void MainWindow::on_horizontalSlider_RotFix_sliderMoved(int position)
{
   static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDispatcherAttribute.coffRotFix = position / 10.f;
}


void MainWindow::on_horizontalSlider_RotLen_sliderMoved(int position)
{
   static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDispatcherAttribute.coffRotLen = position / 10.f;
}

void MainWindow::PushData(QString _str, double _data , int _n_graph)
{

   //ui->widget_Plot->graph(_n_graph)->addData(Numer, _data);

    Graph_Data_X[_str].append(Numer);
    Graph_Data_Y[_str].append(_data);
    ui->widget_Plot->graph(_n_graph)->setData(Graph_Data_X[_str], Graph_Data_Y[_str]);
}

void MainWindow::PushData2(QString _str, float _data, int _n_graph)
{
    ui->widget_Plot->graph(_n_graph)->setData(Graph_Data_X[_str], Graph_Data_Y[_str]);
}

void MainWindow::ClearData(QString _str, int _n_graph)
{
    Graph_Data_X[_str].clear();
    Graph_Data_Y[_str].clear();

   // ui->widget_Plot->graph(_n_graph)->data().clear();
}


void MainWindow::on_pushButton_LQR_clicked()
{

    if((m_IsTracking))
    {
       m_IsTracking = false;
       ui->pushButton_LQR->setText("Tracking - ON");
    }
    else
    {
       m_IsTracking = true;
       ui->pushButton_LQR->setText("Tracking - OFF");
    }
    static_cast<SceneEngineRobocar*>(ui->widget->scene())->m_IsDynamic_LQR = !m_IsTracking;
}


void MainWindow::on_horizontalSlider_speedPoint_sliderMoved(int position)
{
  static_cast<SceneEngineRobocar*>(ui->widget->scene())->speed_point = position;
}


void MainWindow::on_spinBox_extrem_MIN_MAX_valueChanged(int arg1)
{
    static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDispatcherAttribute.extrem_MIN_MAX = arg1;
}


void MainWindow::on_spinBox_extrem_MIN_MAX_editingFinished()
{
   int arg = ui->spinBox_extrem_MIN_MAX->text().toInt();
   static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDispatcherAttribute.extrem_MIN_MAX = arg;
   qDebug() << arg;
}


void MainWindow::on_spinBox_Minimal_editingFinished()
{
    int arg = ui->spinBox_Minimal->text().toInt();
    static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDispatcherAttribute.minimal = arg;
    qDebug() << arg;
}


void MainWindow::on_checkBox_isCorrectDynamic_toggled(bool checked)
{
    static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->mDispatcherAttribute.isCorrectlyDynamics = checked;
}


void MainWindow::on_pushButton_Clear_clicked()
{
//    /*if(ui->checkBox_AngleYaw->isChecked()) */ ClearData("AngleYaw",0);
//    /*if(ui->checkBox_AngleYaw_Derivative->isChecked())*/  ClearData("AngleYaw_Derivative",1);

//    /*if(ui->checkBox_motor_A->isChecked())*/ ClearData("motor_A",2);
//    /*if(ui->checkBox_motor_B->isChecked())*/ ClearData("motor_B",3);
//    /*if(ui->checkBox_motor_C->isChecked())*/ ClearData("motor_C",4);
//    /*if(ui->checkBox_motor_D->isChecked())*/ ClearData("motor_D",5);

    ui->widget_Plot->replot();

    ClearData("AngleYaw",0);
    ClearData("AngleYaw_Derivative",1);

    ClearData("motor_A",2);
    ClearData("motor_B",3);
    ClearData("motor_C",4);
    ClearData("motor_D",5);


    ClearData("Angle_Gimbal_X",6);
    ClearData("Angle_Gimbal_Y",7);
    ClearData("Angle_Gimbal_Z",8);

    ui->widget_Plot->replot();


    Numer=0;
}

