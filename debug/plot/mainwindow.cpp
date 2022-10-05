#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    drawer = new DynamicDraw(ui->centralwidget,this);
    drawer->allotBuffer(500,0);
    drawer->startDraw(30,100);
    drawer->addData(1);
    drawer->addData2(1);
}

MainWindow::~MainWindow()
{
    delete ui;
}

