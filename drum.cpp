#include "drum.h"
#include "ui_drum.h"

#include <QFileDialog>
#include <QtMath>
#include <QMetaType>

#include <QDebug>


Drum::Drum(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::Drum)
{
    ui->setupUi(this);




    initUi();
    initChart();
    init();

}

Drum::~Drum()
{


    m_StateThread.join();
    m_SendThread.join();
    m_ReceiveThread.join();


    delete ui;
}

void Drum::initUi()
{

    ui->label_can1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_can2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_can3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_l1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_l2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_l3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_l4->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_r1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_r2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_r3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_r4->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_waist->setStyleSheet("background-color: dimgrey; border-radius: 12px;");

    ui->label_home_done_l1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_l2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_l3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_l4->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_r1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_r2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_r3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_r4->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_waist->setStyleSheet("background-color: dimgrey; border-radius: 12px;");

    ui->comboBox_state_demand->setCurrentIndex(-1);
    ui->comboBox_view->setCurrentIndex(-1);
    ui->comboBox_home_joint->setCurrentIndex(-1);
    ui->tabWidget->setCurrentIndex(0);
}

void Drum::init()
{

    m_pTimerUI = new QTimer();
    m_pTimerUI->setInterval(300);

    m_pCanManager = new CanManager(m_Motors);
    m_pPathManager = new PathManager(m_SystemState,
                                     *m_pCanManager,
                                     m_Motors);

    m_pTestManager = new TestManager(m_SystemState,
                                     *m_pCanManager,
                                     m_Motors);

    m_pHomeManager = new HomeManager(m_SystemState,
                                     *m_pCanManager,
                                     m_Motors);

    m_pDrumRobot = new DrumRobot(m_SystemState,
                                 *m_pCanManager,
                                 *m_pPathManager,
                                 *m_pHomeManager,
                                 *m_pTestManager,
                                 m_Motors);


    initConnections();

    m_StateThread = std::thread(&DrumRobot::stateMachine, m_pDrumRobot);
    m_SendThread = std::thread(&DrumRobot::sendLoopForThread, m_pDrumRobot);
    m_ReceiveThread = std::thread(&DrumRobot::recvLoopForThread, m_pDrumRobot);



    m_pTimerUI->start();



}

void Drum::initConnections()
{
    QObject::connect(&m_pDrumRobot->m_Signals, &Signals::stateChanged, this, &Drum::setState);
    //QObject::connect(&m_pDrumRobot->m_Signals, &Signals::motorInfosUpdated, this, &Drum::setMotorInfos);
    QObject::connect(&m_pDrumRobot->m_Signals, &Signals::canStateChanged, this, &Drum::setCanState);

    QObject::connect(m_pTimerUI, &QTimer::timeout, this, &Drum::updateUI);
    QObject::connect(ui->comboBox_state_demand, &QComboBox::currentTextChanged, this, &Drum::on_comboBox_state_demand_currentTextChanged);
    QObject::connect(ui->pushButton_music, &QPushButton::clicked, this, &Drum::on_pushButton_music_clicked);
    //QObject::connect(ui->pushButton_start, SIGNAL(clicked()), this, SLOT(on_pushButton_start_clicked()));
    QObject::connect(ui->pushButton_start, &QPushButton::clicked, this, &Drum::on_pushButton_start_clicked);
    QObject::connect(ui->comboBox_view, &QComboBox::currentTextChanged, this, &Drum::on_comboBox_view_currentIndexChanged);
    QObject::connect(ui->pushButton_all_home, &QPushButton::clicked, this, &Drum::on_pushButton_all_home_clicked);
    QObject::connect(ui->pushButton_home, &QPushButton::clicked, this, &Drum::on_pushButton_home_clicked);

    QObject::connect(&m_pDrumRobot->m_Signals, &Signals::homingDone, this, &Drum::setHomeDone);

}

void Drum::initChart()
{
    m_pChartPos = new QChart();
    m_pSeriesPos = new QLineSeries(m_pChartPos);
    //m_pSeriesPos->setName("Pos");
    // QPen red(Qt::red);
    // red.setWidth(3);
    // m_pSeriesPos->setPen(red);
    m_pAxisPosX = new QValueAxis();
    m_pAxisPosY = new QValueAxis();
    m_pAxisPosX->setRange(0, 100);
    m_pAxisPosY->setRange(-10, 100);

    m_pChartPos->addSeries(m_pSeriesPos);
    m_pChartPos->createDefaultAxes();
    m_pChartPos->setAxisX(m_pAxisPosX, m_pSeriesPos);
    m_pChartPos->setAxisY(m_pAxisPosY, m_pSeriesPos);
    // m_pChartPos->axisX()->setRange(0, 100);
    // m_pChartPos->axisY()->setRange(-10, 100);
    m_pChartPos->setAnimationOptions(QChart::AllAnimations);

    ui->graphicsView_pos->setChart(m_pChartPos);
    ui->graphicsView_pos->setRenderHints(QPainter::Antialiasing);



}

void Drum::setState()
{
    switch (m_SystemState.main) {

    case Main::SystemInit:
        ui->lineEdit_cur_state->setText("SystemInit");
        break;
    case Main::Ideal:
        ui->lineEdit_cur_state->setText("IDLE");
        break;
    case Main::Homing:
        ui->lineEdit_cur_state->setText("Homing");
        break;

    case Main::Tune:
        ui->lineEdit_cur_state->setText("Tune");
        break;

    case Main::Perform:
        ui->lineEdit_cur_state->setText("Perform");
        break;

    case Main::Check:
        ui->lineEdit_cur_state->setText("Check");
        break;

    case Main::Shutdown:
        ui->lineEdit_cur_state->setText("Shutdown");
        break;

    case Main::Back:
        ui->lineEdit_cur_state->setText("Back");
        break;

    case Main::Ready:
        ui->lineEdit_cur_state->setText("Ready");
        break;

    }

}

void Drum::setMotorInfos()
{


    if (m_Motors.find("L_arm1") != m_Motors.end()) {
        ui->lineEdit_pos_l1->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm1"]->currentPos), 'f', 2));
        ui->lineEdit_vel_l1->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm1"]->currentVel), 'f', 2));
        ui->lineEdit_tor_l1->setText(QString::number(m_Motors["L_arm1"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_l1->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm1"]->desPos), 'f', 2));
    }

    if (m_Motors.find("L_arm2") != m_Motors.end()) {
        ui->lineEdit_pos_l2->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm2"]->currentPos), 'f', 2));
        ui->lineEdit_vel_l2->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm2"]->currentVel), 'f', 2));
        ui->lineEdit_tor_l2->setText(QString::number(m_Motors["L_arm2"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_l2->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm2"]->desPos), 'f', 2));
    }

    if (m_Motors.find("L_arm3") != m_Motors.end()) {
        ui->lineEdit_pos_l3->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm3"]->currentPos), 'f', 2));
        ui->lineEdit_vel_l3->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm3"]->currentVel), 'f', 2));
        ui->lineEdit_tor_l3->setText(QString::number(m_Motors["L_arm3"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_l3->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm3"]->desPos), 'f', 2));
    }

    if (m_Motors.find("L_wrist") != m_Motors.end()) {
        ui->lineEdit_pos_l4->setText(QString::number(qRadiansToDegrees(m_Motors["L_wrist"]->currentPos), 'f', 2));
        ui->lineEdit_vel_l4->setText(QString::number(qRadiansToDegrees(m_Motors["L_wrist"]->currentVel), 'f', 2));
        ui->lineEdit_tor_l4->setText(QString::number(m_Motors["L_wrist"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_l4->setText(QString::number(qRadiansToDegrees(m_Motors["L_wrist"]->desPos), 'f', 2));
    }

    if (m_Motors.find("R_arm1") != m_Motors.end()) {
        ui->lineEdit_pos_r1->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm1"]->currentPos), 'f', 2));
        ui->lineEdit_vel_r1->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm1"]->currentVel), 'f', 2));
        ui->lineEdit_tor_r1->setText(QString::number(m_Motors["R_arm1"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_r1->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm1"]->desPos), 'f', 2));
    }

    if (m_Motors.find("R_arm2") != m_Motors.end()) {
        ui->lineEdit_pos_r2->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm2"]->currentPos), 'f', 2));
        ui->lineEdit_vel_r2->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm2"]->currentVel), 'f', 2));
        ui->lineEdit_tor_r2->setText(QString::number(m_Motors["R_arm2"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_r2->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm2"]->desPos), 'f', 2));
    }

    if (m_Motors.find("R_arm3") != m_Motors.end()) {
        ui->lineEdit_pos_r3->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm3"]->currentPos), 'f', 2));
        ui->lineEdit_vel_r3->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm3"]->currentVel), 'f', 2));
        ui->lineEdit_tor_r3->setText(QString::number(m_Motors["R_arm3"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_r3->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm3"]->desPos), 'f', 2));
    }

    if (m_Motors.find("R_wrist") != m_Motors.end()) {
        ui->lineEdit_pos_r4->setText(QString::number(qRadiansToDegrees(m_Motors["R_wrist"]->currentPos), 'f', 2));
        ui->lineEdit_vel_r4->setText(QString::number(qRadiansToDegrees(m_Motors["R_wrist"]->currentVel), 'f', 2));
        ui->lineEdit_tor_r4->setText(QString::number(m_Motors["R_wrist"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_r4->setText(QString::number(qRadiansToDegrees(m_Motors["R_wrist"]->desPos), 'f', 2));
    }

    if (m_Motors.find("waist") != m_Motors.end()) {
        ui->lineEdit_pos_waist->setText(QString::number(qRadiansToDegrees(m_Motors["waist"]->currentPos), 'f', 2));
        ui->lineEdit_vel_waist->setText(QString::number(qRadiansToDegrees(m_Motors["waist"]->currentVel), 'f', 2));
        ui->lineEdit_tor_waist->setText(QString::number(m_Motors["waist"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_waist->setText(QString::number(qRadiansToDegrees(m_Motors["waist"]->desPos), 'f', 2));
    }



}

void Drum::setCanState()
{

    int size = m_pCanManager->ifnames.size();

    if (m_pCanManager->isConnected[m_pCanManager->ifnames[0]]) {
        ui->label_can1->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
    }
    else {
        ui->label_can1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    }

    if (m_pCanManager->isConnected[m_pCanManager->ifnames[1]]) {
        ui->label_can2->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
    }
    else {
        ui->label_can2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    }

    if (m_pCanManager->isConnected[m_pCanManager->ifnames[2]]) {
        ui->label_can3->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
    }
    else {
        ui->label_can3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    }
}

void Drum::setHomeDone()
{
    if (m_Motors.find("L_arm1") != m_Motors.end()) {
        ui->label_home_done_l1->setStyleSheet("");
    }
}

void Drum::on_comboBox_state_demand_currentTextChanged(const QString &arg1)
{
    if (arg1 == "") {
        return;
    }

    if (arg1 == "HOME") {
        m_pDrumRobot->m_Input = "h";
    }
    else if (arg1 == "TUNE") {
        m_pDrumRobot->m_Input = "t";
    }
    else if (arg1 == "READY") {
        m_pDrumRobot->m_Input = "r";
    }
    else if (arg1 == "CHECK") {
        m_pDrumRobot->m_Input = "c";
    }
    else if (arg1 == "SHUTDOWN") {
        m_pDrumRobot->m_Input = "s";
    }
    else if (arg1 == "PERFORM") {
        m_pDrumRobot->m_Input = "p";
    }
    else if (arg1 == "BACK") {
        m_pDrumRobot->m_Input = "b";
    }

}


void Drum::on_pushButton_music_clicked()
{
    QString filePath = QFileDialog::getOpenFileName(this, "open File", "./", "File (*.*);;Text files(*.txt)");

}


void Drum::on_pushButton_start_clicked()
{
    int test = 1;

}




void Drum::on_comboBox_view_currentIndexChanged(const QString &name)
{
    m_pSeriesPos->clear();

}

void Drum::updateUI()
{
    // qreal x = m_pSeriesPos->count();
    // qreal y = qrand() %100;
    // m_pSeriesPos->append(x, y);
    // //m_pChartPos->axisX()->setRange(x-10, x+10);
    // if (x > 100) {
    //     m_pSeriesPos->clear();
    // }


    //m_pChartPos->axisY()->setRange(y-10, y+10);
    // m_pSeriesPos->append(m_value,m_value+1);

    // m_pChartPos->addSeries(m_pSeriesPos);
    // //m_pChartPos->createDefaultAxes();
    // ui->graphicsView_pos->update();
    // m_value += 1;

    // m_pSeriesPos->append(m_value, m_value +1);
    // m_pChartPos->update();

    // m_value += 1;


    std::string joint;
    joint = ui->comboBox_view->currentText().toStdString();
    if (m_Motors.find(joint) != m_Motors.end()) {
        //m_PosChart.append();
        qreal x = m_pSeriesPos->count();
        m_pSeriesPos->append(x, qRadiansToDegrees(m_Motors[joint]->currentPos));
        //m_pSeriesPos->append(i, );
        if (x >100) {
            m_pSeriesPos->clear();
        }
    }

    setMotorInfos();
}


void Drum::on_pushButton_all_home_clicked()
{
    m_pHomeManager->m_MotorName = "all";
}


void Drum::on_pushButton_home_clicked()
{
    m_pHomeManager->m_MotorName = ui->comboBox_home_joint->currentText().toStdString();
}

