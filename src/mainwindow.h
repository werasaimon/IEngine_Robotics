#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QMap>
#include "Engine/engine.hpp"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);

private slots:
    void Update();

    void on_pushButton_Timer_clicked();

    void on_checkBox_Range_toggled(bool checked);

    void on_checkBox_Gimbal_toggled(bool checked);

    void on_pushButton_simulate_clicked();

    void on_horizontalSlider_Deverative0_sliderMoved(int position);

    void on_horizontalSlider_Deverative1_sliderMoved(int position);

    void on_horizontalSlider_Deverative2_sliderMoved(int position);

    void on_horizontalSlider_RotDev_sliderMoved(int position);

    void on_horizontalSlider_RotFix_sliderMoved(int position);

    void on_horizontalSlider_RotLen_sliderMoved(int position);

    void on_pushButton_LQR_clicked();

    void on_horizontalSlider_speedPoint_sliderMoved(int position);

    void on_spinBox_extrem_MIN_MAX_valueChanged(int arg1);

    void on_spinBox_extrem_MIN_MAX_editingFinished();

    void on_spinBox_Minimal_editingFinished();

    void on_checkBox_isCorrectDynamic_toggled(bool checked);

    void on_pushButton_Clear_clicked();

private:

    // Порт обмена данными
    quint16 m_Port;
    int MAX = 100;//
    double Numer;
    int N;

    QMap<QString,QVector<double>> Graph_Data_X;
    QMap<QString,QVector<double>> Graph_Data_Y;

    void PushData(QString _str, double _data, int _n_graph);
    void PushData2(QString _str, float _data, int _n_graph);
    void ClearData(QString _str, int _n_graph);

    Ui::MainWindow *ui;
    QTimer *timer;
};

#endif // MAINWINDOW_H
