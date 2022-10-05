#include "mainwindow.h"
#include "ui_mainwindow.h"
#include<QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    if (!lcm.good())
    {

    }

    this->setWindowTitle("3 dimention draw");

    init();

    lcm.subscribe("debug",&MainWindow::handle_function,this);

    connect(&draw_timer,&QTimer::timeout,this,&MainWindow::drawPoint);
    connect(&communicate_timer,&QTimer::timeout,this,&MainWindow::handle);

    start();
}
void MainWindow::handle()
{
    lcm.handle();
}
MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::handle_function(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const exlcm::example_t* msg)
{
    if(points.size()>=80)
    {
        points.erase(points.begin());
    }
    QVector3D point(msg->pose[0],msg->pose[1],msg->pose[2]);
    points.push_back(point);
}
void MainWindow::init()
{
    m_3Dgraph = new Q3DScatter();
    QWidget *container = QWidget::createWindowContainer(m_3Dgraph);

    QHBoxLayout *hLayout = new QHBoxLayout();
    hLayout->addWidget(container, 1);
    ui->centralwidget->setLayout(hLayout);

    QScatterDataProxy *proxy = new QScatterDataProxy(); //数据代理
    m_3Dseries = new QScatter3DSeries(proxy);//创建序列
    m_3Dseries->setMeshSmooth(true);
    //创建坐标轴
    m_3Dgraph->scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetBehindLow);
    m_3Dgraph->axisX()->setTitle("axis X");
    m_3Dgraph->axisX()->setTitleVisible(true);
    m_3Dgraph->axisX()->setRange(-35,60);
    m_3Dgraph->axisY()->setTitle("axis Y");
    m_3Dgraph->axisY()->setTitleVisible(true);
    m_3Dgraph->axisY()->setRange(-3,4);
    m_3Dgraph->axisZ()->setTitle("axis Z");
    m_3Dgraph->axisZ()->setTitleVisible(true);
    m_3Dgraph->axisZ()->setRange(0,35);
    m_3Dgraph->activeTheme()->setLabelBackgroundEnabled(false);
    m_3Dgraph->activeTheme()->setBackgroundColor(QColor(255,0,0));//设置背景色

    m_3Dseries->setMesh(QAbstract3DSeries::MeshSphere);//数据点为圆球
    m_3Dseries->setSingleHighlightColor(QColor(120,120,120));//设置点选中时的高亮颜色
    m_3Dseries->setBaseColor(QColor(0,255,255));//设置点的颜色
    m_3Dseries->setItemSize(0.05);//设置点的大小

    m_3Dgraph->addSeries(m_3Dseries);

    points.reserve(80);
    initPoints();
    m_3Dgraph->show();
}
void MainWindow::initPoints()
{
    for(int i=0;i<80;i++)
    {
        points.push_back(QVector3D(i,i,i));
    }
}
void MainWindow::drawPoint()
{
    QScatterDataArray *dataArray = new QScatterDataArray();
    dataArray->resize(points.size());
    QScatterDataItem *ptrToDataArray = &dataArray->first();

    double max_point[3] = {-9999,-9999,-9999};
    double min_point[3] = {9999,9999,9999};
    for (int i = 0;i < points.size();i++ )
    {
        QVector3D temp_point(points[i][0],points[i][1],points[i][2]);
        ptrToDataArray->setPosition(temp_point);
        ptrToDataArray++;

        for(int j=0;j<3;j++)
        {
            max_point[j] = max_point[j]>points[i][j]?max_point[j]:points[i][j];
            min_point[j] = min_point[j]<points[i][j]?min_point[j]:points[i][j];
        }
    }
    m_3Dgraph->axisX()->setRange(min_point[0]-0.3,max_point[0]+0.3);
    m_3Dgraph->axisY()->setRange(min_point[1]-0.3,max_point[1]+0.3);
    m_3Dgraph->axisZ()->setRange(min_point[2]-0.3,max_point[2]+0.3);
    m_3Dseries->dataProxy()->resetArray(dataArray);//更新散点
    m_3Dgraph->show();
}
void MainWindow::start()
{
    if(communicate_timer.isActive())
        communicate_timer.stop();
    if(draw_timer.isActive())
        draw_timer.stop();
    communicate_timer.start(8);
    draw_timer.start(8);
}

