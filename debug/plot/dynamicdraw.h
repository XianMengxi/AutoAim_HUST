#ifndef DYNAMICDRAW_H
#define DYNAMICDRAW_H

#include <QObject>
#include"qcustomplot.h"
#include<QTimer>
#include<QQueue>
#include"lcm/lcm-cpp.hpp"
#include"example_t.hpp"
#include<QDebug>

class DynamicDraw : public QObject
{
    Q_OBJECT
public:
    explicit DynamicDraw(QObject *parent = nullptr);
public:
    DynamicDraw(QCustomPlot* customPlot, QObject* parent = nullptr);
    ~DynamicDraw();
    void allotBuffer(u_short size, float initialValue = 0);
    void addData(float data);
    void startDraw(u_short lineRefreshTime = 30, u_short axisRefreshTime = 100);
    void magnify();
    void addData2(float data);
    void shrink();
    void handle_function(const lcm::ReceiveBuffer* rbuf,
                         const std::string& chan,
                         const exlcm::example_t* msg);
private slots:
    void on_refreshLine_timeOut();
    void on_refreshAxis_timeOut();
    void on_refrashData_timeout();
private:
    QCustomPlot* m_draw;
    QTimer m_lineTimer;
    QTimer m_axisTimer;
    QTimer m_communicateTimer;
    QQueue<float> m_dataQueue;
    QQueue<float> m_dataQueue2;
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
