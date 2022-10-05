#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVector3D>
#include <QDebug>
#include <QPointF>
#include <QtDataVisualization>
#include <QAbstract3DInputHandler>
#include <QSplitter>
#include<Q3DScatter>
#include"eigen3/Eigen/Core"
#include"example_t.hpp"
#include"lcm/lcm-cpp.hpp"
#include<QTimer>
#include<QMessageBox>
#include<QHBoxLayout>

using namespace QtDataVisualization;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


private:
    Ui::MainWindow *ui;
    void init();
    void handle_function(const lcm::ReceiveBuffer* rbuf,
                                const std::string& chan,
                                const exlcm::example_t* msg);

    Q3DScatter *m_3Dgraph;
    QScatter3DSeries *m_3Dseries;

    lcm::LCM lcm;

    QTimer communicate_timer;
    QTimer draw_timer;

    QVector<QVector3D> points;

    void start();
    void initPoints();
private slots:
    void handle();
    void drawPoint();

};
#endif // MAINWINDOW_H
