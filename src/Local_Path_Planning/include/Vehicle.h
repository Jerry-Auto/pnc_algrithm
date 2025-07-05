#ifndef VEHICLE_H
#define VEHICLE_H

#include <QObject>
#include <QGraphicsItem>
#include <QColor>
#include <QPointF>
#include <QSizeF>
#include <QVector2D>
#include <QTimer>

class WorldMap;

class Vehicle : public QObject, public QGraphicsItem {
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    // 车辆参数结构体
    struct Parameters {
        double mass;          // 质量 (kg)
        double lf;            // 质心到前轴距离 (m)
        double lr;            // 质心到后轴距离 (m)
        double iz;            // 转动惯量 (kg·m²)
        double cf;            // 前轮侧偏刚度 (N/rad)
        double cr;            // 后轮侧偏刚度 (N/rad)
        double max_steer;     // 最大前轮转角 (rad)
        double max_accel;     // 最大加速度 (m/s²)
        double max_decel;     // 最大减速度 (m/s²)
    };

    // 车辆状态结构体
    struct State {
        QPointF position;     // 位置 (m)
        double velocity;      // 速度 (m/s)
        double heading;       // 航向角 (rad)
        double steer_angle;   // 前轮转角 (rad)
        double slip_angle;    // 滑移角 (rad)
    };

    explicit Vehicle(WorldMap* map, const Parameters& params, QObject *parent = nullptr);
    ~Vehicle();

    // 获取车辆状态
    State getState() const;
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

    // 控制接口
    void setControlInput(double throttle, double steer); //  throttle: [0,1], steer: [-1,1]

    // 设置显示属性
    void setstate(QPointF position,double velocity,double heading,double steer_angle,double slip_angle);
    void setSize(const QSizeF& size);
    void setColor(const QColor& color);
    void setTrajectoryVisible(bool visible);
    void setSensorsVisible(bool visible);

    // 感知相关
    QVector<QPointF> getObstaclesInRange(double range) const;

signals:
    void stateChanged();

private:
    void updateState(double dt);
    void updateGraphics();
    void updateTrajectory();
    void updateSensors();

    WorldMap* m_map;
    Parameters m_params;
    State m_state;
    
    // 控制输入
    double m_throttle;  // [0, 1]
    double m_steer;     // [-1, 1]
    
    // 图形属性
    QSizeF m_size;
    QColor m_color;
    bool m_trajectoryVisible;
    bool m_sensorsVisible;
    
    // 轨迹记录
    QVector<QPointF> m_trajectory;
    
    // 仿真定时器
    QTimer m_updateTimer;
    
    // 传感器范围
    double m_sensorRange;
};

#endif // VEHICLE_H