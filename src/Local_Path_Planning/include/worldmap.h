#ifndef WORLDMAP_H
#define WORLDMAP_H

#include <QObject>
#include <QGraphicsScene>
#include <QVector>
#include <QPointF>
#include <QColor>

class Vehicle;

class WorldMap : public QObject {
    Q_OBJECT
public:
    explicit WorldMap(QObject *parent = nullptr);
    ~WorldMap();
    
    // 地图管理
    void setScene(QGraphicsScene* scene);
    void addObstacle(const QPointF& position, double radius, const QColor& color = Qt::red);
    void addVehicle(Vehicle* vehicle);
    void removeVehicle(Vehicle* vehicle);
    
    // 感知相关
    QVector<QPointF> getObstaclesInRange(const QPointF& position, double range) const;
    QVector<Vehicle*> getVehiclesInRange(const QPointF& position, double range) const;
    
    // 坐标转换
    QPointF metersToPixels(const QPointF& meters) const;
    QPointF pixelsToMeters(const QPointF& pixels) const;
    
    // 地图参数
    void setScale(double pixelsPerMeter);
    double scale() const;
    
private:
    QGraphicsScene* m_scene;
    QVector<QPair<QPointF, double>> m_obstacles; // (position, radius)
    QVector<Vehicle*> m_vehicles;
    double m_scale; // 像素/米
};

#endif // WORLDMAP_H