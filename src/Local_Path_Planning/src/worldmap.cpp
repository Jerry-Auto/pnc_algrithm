#include "worldmap.h"
#include "Vehicle.h"
#include <QGraphicsEllipseItem>
#include <QDebug>

WorldMap::WorldMap(QObject *parent) : QObject(parent), m_scene(nullptr), m_scale(50.0) {
    // 默认比例: 1米 = 50像素
}

WorldMap::~WorldMap() {
    // 车辆由外部管理，不在这里删除
}

void WorldMap::setScene(QGraphicsScene* scene) {
    m_scene = scene;
}

void WorldMap::addObstacle(const QPointF& position, double radius, const QColor& color) {
    if (!m_scene) return;
    
    // 转换为像素坐标
    QPointF pixelPos = metersToPixels(position);
    double pixelRadius = radius * m_scale;
    
    // 创建图形项
    QGraphicsEllipseItem* obstacle = new QGraphicsEllipseItem(
        pixelPos.x() - pixelRadius, 
        pixelPos.y() - pixelRadius, 
        pixelRadius * 2, 
        pixelRadius * 2
    );
    obstacle->setPen(QPen(color, 1));
    obstacle->setBrush(QBrush(color.lighter(120)));
    m_scene->addItem(obstacle);
    
    // 存储障碍物信息
    m_obstacles.append(qMakePair(position, radius));
}

void WorldMap::addVehicle(Vehicle* vehicle) {
    if (vehicle && !m_vehicles.contains(vehicle)) {
        m_vehicles.append(vehicle);
        if (m_scene) {
            m_scene->addItem(vehicle);
        }
    }
}

void WorldMap::removeVehicle(Vehicle* vehicle) {
    if (vehicle) {
        m_vehicles.removeOne(vehicle);
        if (m_scene) {
            m_scene->removeItem(vehicle);
        }
    }
}

QVector<QPointF> WorldMap::getObstaclesInRange(const QPointF& position, double range) const {
    QVector<QPointF> obstacles;
    for (const auto& obstacle : m_obstacles) {
        double dist = QLineF(position, obstacle.first).length();
        if (dist <= range + obstacle.second) {
            obstacles.append(obstacle.first);
        }
    }
    return obstacles;
}

QVector<Vehicle*> WorldMap::getVehiclesInRange(const QPointF& position, double range) const {
    QVector<Vehicle*> vehicles;
    for (Vehicle* vehicle : m_vehicles) {
        if (vehicle == nullptr) continue;
        
        double dist = QLineF(position, vehicle->getState().position).length();
        if (dist <= range) {
            vehicles.append(vehicle);
        }
    }
    return vehicles;
}

QPointF WorldMap::metersToPixels(const QPointF& meters) const {
    return QPointF(meters.x() * m_scale, -meters.y() * m_scale); // y轴反转
}

QPointF WorldMap::pixelsToMeters(const QPointF& pixels) const {
    return QPointF(pixels.x() / m_scale, -pixels.y() / m_scale); // y轴反转
}

void WorldMap::setScale(double pixelsPerMeter) {
    m_scale = pixelsPerMeter;
    // 通知所有车辆更新显示
    for (Vehicle* vehicle : m_vehicles) {
        if (vehicle) {
            vehicle->update();
        }
    }
}

double WorldMap::scale() const {
    return m_scale;
}