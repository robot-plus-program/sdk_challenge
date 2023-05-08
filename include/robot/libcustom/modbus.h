#ifndef MODBUS_H
#define MODBUS_H

#include <QObject>
#include <QMainWindow>

#include <QtNetwork>
#include <QVector>
#include <QModbusClient>
#include <QModbusTcpClient>
#include <QModbusReply>

#include <time.h>


class modbus : public QMainWindow
{
//    QModbusClient modbusClient;
//    QModbusDataUnit writeUnit=QModbusDataUnit(static_cast<QModbusDataUnit::RegisterType>(4),0x0801,8);
//    QModbusDataUnit readUnit=QModbusDataUnit(static_cast<QModbusDataUnit::RegisterType>(4),0x0002,3);
//    QVector<quint16> m_holdingRegisters;
    Q_OBJECT
    QTimer modbustimer;

    QModbusTcpClient *zimmer;

    QModbusDataUnit writeUnit=QModbusDataUnit(static_cast<QModbusDataUnit::RegisterType>(4),0x0801,8);
    QModbusDataUnit readUnit=QModbusDataUnit(static_cast<QModbusDataUnit::RegisterType>(4),0x0002,3);
    QVector<quint16> m_holdingRegisters;
    quint16 StatusWord,Diagnosis,ActualPosition;

    QThread *Qthr;


public:
    int a=0;
    explicit modbus(QWidget *parent = nullptr);
    ~modbus();
    void run();
private:

public slots:
     void test();
    int test2();
};

#endif // MODBUS_H
