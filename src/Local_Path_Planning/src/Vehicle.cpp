#include "Vehicle.h"
#include "worldmap.h"
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QBrush>
#include <cmath>
#include <QDebug>

Vehicle::Vehicle(WorldMap* map, const Parameters& params, QObject *parent)
    : QObject(parent), m_map(map), m_params(params), m_sensorRange(10.0) {
    // 初始状态
    m_state.position = QPointF(0, 0);
    m_state.velocity = 0;
    m_state.heading = 0;
    m_state.steer_angle = 0;
    m_state.slip_angle = 0;
    
    // 初始控制输入
    m_throttle = 0;
    m_steer = 0;
    
    // 默认图形属性
    m_size = QSizeF(4.0, 2.0); // 单位:米
    m_color = Qt::blue;
    m_trajectoryVisible = false;
    m_sensorsVisible = false;
    
    // 设置图形项标志
    setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    
    // 启动更新定时器
    connect(&m_updateTimer, &QTimer::timeout, [this]() {
        updateState(0.05); // 20Hz更新
        updateGraphics();
        emit stateChanged();
    });
    m_updateTimer.start(50); // 50ms
}

Vehicle::~Vehicle() {
    m_updateTimer.stop();
}

Vehicle::State Vehicle::getState() const {
    return m_state;
}

QRectF Vehicle::boundingRect() const {
    double length = m_size.height(); // 车长
    double width = m_size.width();   // 车宽
    double wheelOffset = 0.5;        // 车轮超出车身的额外空间（米）

    // 返回包含车身和车轮的矩形
    return QRectF(
        -width/2 - wheelOffset,   // left
        -length/2 - wheelOffset,  // top
        width + 2*wheelOffset,    // width
        length + 2*wheelOffset    // height
    );
}

void Vehicle::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->setRenderHint(QPainter::Antialiasing);

    // 1. 保存当前画笔状态
    painter->save();

    // 2. 移动到车辆中心（确保旋转围绕中心）
    // （QGraphicsItem 默认以 (0,0) 为中心，无需额外移动）

    // 3. 应用旋转（航向角，单位：弧度 → 角度）
    painter->rotate(m_state.heading * 180 / M_PI);

    // 4. 绘制车身（蓝色）
    painter->setBrush(QBrush(m_color));
    painter->setPen(QPen(Qt::black, 0.1));
    double length = m_size.height();
    double width = m_size.width();
    painter->drawRoundedRect(QRectF(-length/2, -width/2, length, width), 0.1, 0.1);

    // 5. 恢复画笔状态（旋转和移动）
    painter->restore();
}

void Vehicle::setControlInput(double throttle, double steer) {
    // 限制输入范围
    m_throttle = qBound(0.0, throttle, 1.0);
    m_steer = qBound(-1.0, steer, 1.0) * m_params.max_steer;
}

void Vehicle::setSize(const QSizeF& size) {
    m_size = size;
    update();
}

void Vehicle::setColor(const QColor& color) {
    m_color = color;
    update();
}

void Vehicle::setTrajectoryVisible(bool visible) {
    m_trajectoryVisible = visible;
}

void Vehicle::setSensorsVisible(bool visible) {
    m_sensorsVisible = visible;
    update();
}

void Vehicle::setstate(QPointF position,double velocity,double heading,double steer_angle,double slip_angle)
{
    m_state.position = position;
    m_state.velocity = velocity;
    m_state.heading = heading;
    m_state.steer_angle = steer_angle;
    m_state.slip_angle = slip_angle;
}

QVector<QPointF> Vehicle::getObstaclesInRange(double range) const {
    return m_map->getObstaclesInRange(m_state.position, range);
}

void Vehicle::updateState(double dt) {
    // 二自由度自行车模型
    
    // 计算前后轮速度
    double vx = m_state.velocity * cos(m_state.slip_angle);
    double vy = m_state.velocity * sin(m_state.slip_angle);
    
    // 计算加速度 (简化模型)
    double accel = m_throttle * m_params.max_accel - 
                  (m_throttle < 0 ? m_throttle * m_params.max_decel : 0);
    
    // 计算滑移角变化率
    double slip_dot = (m_params.cr / m_params.mass - m_params.cf / (2 * m_params.mass)) * m_state.velocity - 
                     (m_params.cf + m_params.cr) / (m_params.mass * m_state.velocity) * m_state.slip_angle - 
                     (m_params.cf * m_params.lf - m_params.cr * m_params.lr) / (m_params.mass * m_state.velocity) * m_state.steer_angle;
    
    // 计算航向角变化率
    double heading_dot = m_state.velocity * (m_state.steer_angle - 
                        (m_params.cf * m_params.lf - m_params.cr * m_params.lr) / (m_params.iz * m_state.velocity) * m_state.slip_angle - 
                        (m_params.cf * m_params.lf * m_params.lf + m_params.cr * m_params.lr * m_params.lr) / (m_params.iz * m_state.velocity) * m_state.steer_angle);
    
    // 更新状态
    m_state.velocity += accel * dt;
    if (m_state.velocity < 0) m_state.velocity = 0;
    
    m_state.slip_angle += slip_dot * dt;
    m_state.heading += heading_dot * dt;
    
    // 更新位置
    double dx = vx * cos(m_state.heading) - vy * sin(m_state.heading);
    double dy = vx * sin(m_state.heading) + vy * cos(m_state.heading);
    
    m_state.position += QPointF(dx, dy) * dt;
    
    // 更新前轮转角(低通滤波)
    static const double steer_tau = 0.1; // 时间常数
    m_state.steer_angle += (m_steer - m_state.steer_angle) * dt / steer_tau;
    
    // 更新轨迹
    if (m_trajectoryVisible) {
        m_trajectory.append(m_state.position);
        // 限制轨迹长度
        if (m_trajectory.size() > 1000) {
            m_trajectory.removeFirst();
        }
    }
}

void Vehicle::updateGraphics() {
    // 更新图形项位置和旋转
    setPos(m_state.position);
    setRotation(m_state.heading * 180/M_PI);
    update();
}