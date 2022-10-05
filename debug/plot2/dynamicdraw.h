#ifndef DYNAMICDRAW_H
#define DYNAMICDRAW_H

#include <QObject>
#include"qcustomplot.h"
#include<QTimer>
#include<QQueue>
#include"lcm/lcm-cpp.hpp"
#include"info.hpp"
#include<QDebug>
struct DebugInfo
{
    double time_stamp;
    double position_to_show;
};

class DynamicDraw : public QObject
{
    Q_OBJECT
public:
    explicit DynamicDraw(QObject *parent = nullptr);
public:
    DynamicDraw(QCustomPlot* customPlot, QObject* parent = nullptr);
    ~DynamicDraw();
    void allotBuffer(u_short size, float initialValue = 0);
    void addData(double time_stamp,float data);
    void startDraw(u_short lineRefreshTime = 30, u_short axisRefreshTime = 100);
    void magnify();
    void addData2(double time_stamp,float data);
    void shrink();
    void handle_function(const lcm::ReceiveBuffer* rbuf,
                         const std::string& chan,
                         const exlcm::info* msg);
private slots:
    void on_refreshLine_timeOut();
    void on_refreshAxis_timeOut();
    void on_refrashData_timeout();
private:
    QCustomPlot* m_draw;
    QTimer m_lineTimer;
    QTimer m_axisTimer;
    QTimer m_communicateTimer;
    QQueue<double> m_dataQueue;
    QQueue<double> m_dataQueue2;
    QQueue<double> m_timeQueue;
    QQueue<double> m_timeQueue2;
    u_short m_bufferSize;
    double m_max;
    double m_min;
    float m_magnification;
    QCPGraphDataContainer* m_drawBuffer;
    QCPGraphDataContainer* m_drawBuffer2;
    lcm::LCM lcm;
signals:

public slots:
};

#endif // DYNAMICDRAW_H
