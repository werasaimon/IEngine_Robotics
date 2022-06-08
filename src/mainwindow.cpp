#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QLineEdit>
#include <QLabel>

#include <QFileDialog>

/**/
struct Quaternionn
{
  float w, x, y, z;
};

struct DataDescriptor
{
  int num;
  Quaternion Quat;
  float EulerX;
  float EulerY;
  float EulerZ;

  //--------------//

  float AccBiasX;
  float AccBiasY;
  float AccBiasZ;

  float GyroBiasX;
  float GyroBiasY;
  float GyroBiasZ;

  float MagBiasX;
  float MagBiasY;
  float MagBiasZ;

  float MagScaleX;
  float MagScaleY;
  float MagScaleZ;

  //--------------//

} _dataDescriptor;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("IEngine - Werasaimon");

    timer = new QTimer(this);
    timer2 = new QTimer(this);
    m_socket = new QUdpSocket(this);

    connect(timer, SIGNAL(timeout()), this, SLOT(Update()));
    connect(m_socket, SIGNAL(readyRead()), this, SLOT(readyRead()));

    m_IsConnectUDP = false;

    for(int i=0; i < 6 + 3; ++i)
    {
         ui->widget_Plot->addGraph();
         QPen pen1;
         pen1.setWidth(3);
         pen1.setColor(QColor(rand()%255,rand()%255,rand()%255));
         ui->widget_Plot->graph(i)->setPen(pen1);
    }

    Numer = 0;
    m_Port = 8888;

    // give the axes some labels:
    ui->widget_Plot->xAxis->setLabel("Time");
    ui->widget_Plot->yAxis->setLabel("Data");
    // set axes ranges, so we see all data:
    ui->widget_Plot->xAxis->setRange(0, MAX);
    ui->widget_Plot->yAxis->setRange(-180, 180);



//     connect(ui->pushButton_OnOFF , SIGNAL(clicked()) , this , SLOT(onStartTime()) );
//     connect(timer , SIGNAL(timeout()) , this , SLOT(Updatee()) );

//     connect(ui->horizontalSliderX, SIGNAL(valueChanged(int)), this, SLOT(SliderSpeedX(int)));
//     connect(ui->horizontalSliderY, SIGNAL(valueChanged(int)), this, SLOT(SliderSpeedY(int)));
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

void MainWindow::Updatee()
{
    // отыслаем данные на ROBO_CAR
    QString ipAddresStr ;
    m_socket->writeDatagram( (char*)&data_trransmission , sizeof(DataPacketRemote) , QHostAddress( ipAddresStr ) , m_Port);

    static_cast<SceneEngineRobocar*>(ui->widget->scene())->data_trransmission = data_trransmission;

    char buff[255]="no packet .... \n";
    int nsize = m_socket->readDatagram( (char*)&_dataDescriptor , sizeof(DataDescriptor) );
    buff[nsize]=0;

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

    if((m_IsTracking) && !m_IsConnectUDP)
    {
       m_IsTracking = false;
       ui->pushButton_LQR->setText("Tracking - ON");
       //ui->checkBox_TrackerMove->setCheckable(false);
       //static_cast<SceneEngineRobocar*>(ui->widget->scene())->m_IsTrackingMove = false;
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


void MainWindow::on_pushButton_StartUDP_clicked()
{
    if(m_IsConnectUDP == false)
    {
        bool result = m_socket->bind(QHostAddress("192.168.1.8"), m_Port);
        qDebug() << m_socket->localAddress() << m_socket->localPort();

        if(result)
        {
            m_IsConnectUDP = true;
            qDebug() << "CONNECT";
            ui->pushButton_StartUDP->setText( m_socket->localAddress().toString() + " :" + QString::number(m_socket->localPort()));

            static_cast<SceneEngineRobocar*>(ui->widget->scene())->mTrackerPoints.clear();

            if((!m_IsTracking))
            {
               m_IsTracking = true;
               ui->pushButton_LQR->setText("Tracking - OFF");
               static_cast<SceneEngineRobocar*>(ui->widget->scene())->m_IsDynamic_LQR = !m_IsTracking;
               ui->checkBox_TrackerMove->setChecked(false);
            }
        }
        else
        {
            qDebug() << "FAIL";
            ui->pushButton_StartUDP->setText("FAIL");
        }
    }
    else
    {
        ui->pushButton_StartUDP->setText("Start_UDP");
        m_socket->close();
        m_IsConnectUDP = false;
    }
}

void MainWindow::readyRead()
{

    // when data comes in
      QByteArray buffer;
      buffer.resize(m_socket->pendingDatagramSize());

      QHostAddress sender;
      quint16 senderPort;


//      // отыслаем данные на ROBO_CAR
//      QString ipAddresStr ;
//      m_socket->writeDatagram( (char*)&data_trransmission , sizeof(DataPacketRemote) , QHostAddress( ipAddresStr ) , m_Port);

//      static_cast<SceneEngineRobocar*>(ui->widget->scene())->data_trransmission = data_trransmission;

//      char buff[255]="no packet .... \n";
//      int nsize = m_socket->readDatagram( (char*)&_dataDescriptor , sizeof(DataDescriptor) );
//      buff[nsize]=0;

      // qint64 QUdpSocket::readDatagram(char * data, qint64 maxSize,
      //                 QHostAddress * address = 0, quint16 * port = 0)
      // Receives a datagram no larger than maxSize bytes and stores it in data.
      // The sender's host address and port is stored in *address and *port
      // (unless the pointers are 0).

      m_socket->readDatagram(/*buffer.data()*/(char*)&data_trransmission, buffer.size(), &sender, &senderPort);
      static_cast<SceneEngineRobocar*>(ui->widget->scene())->data_trransmission = data_trransmission;

      qDebug() << "Message from: " << sender.toString();
      qDebug() << "Message port: " << senderPort;
      qDebug() << "Message: " << buffer;
}


void MainWindow::on_actionsave_tracking_triggered()
{

//    QByteArray data;
//    QFile file("in.txt");
//    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
//        return;

//    QTextStream in(&file);
//    // You could use readAll() here, too.
//    while (!in.atEnd())
//    {
//        QString line = in.readLine();
//        data.append(line);
//    }

//    file.close();

        //   // qDebug() << "Save";
        //    QString filename = QFileDialog::getSaveFileName(this, "test sav e name", ".", "Text files (*.txt);" );
        //    qDebug() << "name is : " << filename;
        //    if( !filename.isNull() )
        //      {
        //        qDebug( "%s", filename.toStdString().c_str() );
        //      }

   // timer->stop();

   ui->widget->timer.stop();


    QString filename = QFileDialog::getSaveFileName(nullptr, "test sav e name", ".", "Text files (*.txt);" );;
//    QFile file(filename);
//    if (file.open(QIODevice::ReadWrite))
//    {
//        QTextStream stream(&file);
//        stream << "something" << endl;
//    }

    // Create a new file
      QFile file(filename);
      file.open(QIODevice::WriteOnly | QIODevice::Text);
      QTextStream out(&file);
      out << "This file is generated by Qt\n";


      auto V_array = static_cast<SceneEngineRobocar*>(ui->widget->scene())->mTrackerPoints;

      for(const auto &it : V_array)
      {
          out << it.x << "," << it.y << "," << it.z << "\n";
      }


      // optional, as QFile destructor will already do it:
      file.close();

     // timer->start(10);

      ui->widget->timer.start(10,ui->widget);
}


void MainWindow::on_actionopen_tracking_triggered()
{

    ui->widget->timer.stop();
    QString fileName = QFileDialog::getOpenFileName(this,tr("Open Config"), "", tr("Text Files (*.txt);; CID Files (*.cid)"));


    QFile file(fileName); // this is a name of a file text1.txt sent from main method
    if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    {

        static_cast<SceneEngineRobocar*>(ui->widget->scene())->mTrackerPoints.clear();
        static_cast<SceneEngineRobocar*>(ui->widget->scene())->num = 0;


        qDebug() << fileName;
        QTextStream in(&file);
        QString line = in.readLine();


        bool isInit = true; Vector3 P;
        auto &V_array = static_cast<SceneEngineRobocar*>(ui->widget->scene())->mTrackerPoints;
        while(!in.atEnd())
        {
           QString line = in.readLine();

           QRegExp rx("[,]");// match a comma or a space
           QStringList list = line.split(rx, QString::SkipEmptyParts);

           auto x = list[0].toDouble();
           auto y = list[1].toDouble();
           auto z = list[2].toDouble();
           V_array.push_back(Vector3(x,y,z));
           qDebug() << x << y << z;
           qDebug() << line;

           if(isInit)
           {
              isInit = false;
              P = Vector3(x,y,z);
           }
        }

        static_cast<SceneEngineRobocar*>(ui->widget->scene())->m_EndPoint = P;
        static_cast<SceneEngineRobocar*>(ui->widget->scene())->m_pickPoint =
        //static_cast<SceneEngineRobocar*>(ui->widget->scene())->m_PointS =
        static_cast<SceneEngineRobocar*>(ui->widget->scene())->m_EndPoint + Vector3::Y * 3;


        static_cast<SceneEngineRobocar*>(ui->widget->scene())->m_PointS =
        static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->physBody_Base->GetTransform().GetPosition();

       // static_cast<SceneEngineRobocar*>(ui->widget->scene())->mRoboCar->physBody_Base->SetCenterOfMassWorld(P+ Vector3::Y * 3);

        file.close();
    }

    ui->widget->timer.start(10,ui->widget);
}



void MainWindow::on_checkBox_TrackerMove_toggled(bool checked)
{
    qDebug() << "Cheked Tracking Move : " ;

    static_cast<SceneEngineRobocar*>(ui->widget->scene())->mTrackerPoints.clear();
    static_cast<SceneEngineRobocar*>(ui->widget->scene())->num = 0;
    static_cast<SceneEngineRobocar*>(ui->widget->scene())->m_IsTrackingMove = checked;
}

