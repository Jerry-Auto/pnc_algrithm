#include <QApplication>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QTimer>
#include <QRandomGenerator>
#include <QScreen>
#include <QWheelEvent>
#include "worldmap.h"
#include "Vehicle.h"

// 自定义 QGraphicsView 类，添加缩放功能
class ZoomableGraphicsView : public QGraphicsView {
public:
    ZoomableGraphicsView(QWidget *parent = nullptr) : QGraphicsView(parent) {
        setDragMode(QGraphicsView::ScrollHandDrag); // 允许拖动视图
        setTransformationAnchor(QGraphicsView::AnchorUnderMouse); // 缩放以鼠标位置为中心
        setResizeAnchor(QGraphicsView::AnchorUnderMouse);
    }

protected:
    void wheelEvent(QWheelEvent *event) override {
        const double scaleFactor = 1.1; // 缩放因子（每次滚轮滚动 10%）

        if (event->angleDelta().y() > 0) {
            // 滚轮向上滚动 → 放大
            scale(scaleFactor, scaleFactor);
        } else {
            // 滚轮向下滚动 → 缩小
            scale(1.0 / scaleFactor, 1.0 / scaleFactor);
        }
    }
};

int main(int argc, char *argv[]) {
    qputenv("QT_QPA_PLATFORM", "xcb"); // 强制使用 X11
    QApplication app(argc, argv);

    // 获取屏幕尺寸
    QScreen *screen = QGuiApplication::primaryScreen();
    QRect screenGeometry = screen->geometry();
    int screenWidth = screenGeometry.width();
    int screenHeight = screenGeometry.height();

    // 创建场景
    QGraphicsScene scene;

    // 创建自定义视图（不传场景）
    ZoomableGraphicsView view;
    view.setScene(&scene); // 关键：用 setScene() 设置场景
    view.setRenderHint(QPainter::Antialiasing);
    view.setWindowTitle("Vehicle Simulation with World Map");

    // 创建世界地图
    WorldMap worldMap;
    worldMap.setScene(&scene);
    worldMap.setScale(50.0); // 1米 = 50像素
    
    // 计算合适的初始视图范围（考虑屏幕比例）
    qreal aspectRatio = static_cast<qreal>(screenWidth) / screenHeight;
    qreal mapWidth = 100; // 默认地图宽度（米）
    qreal mapHeight = mapWidth / aspectRatio;
    
    // 添加网格背景
    for (int x = -static_cast<int>(mapWidth/2); x <= mapWidth/2; x += 5) {
        QPointF p1 = worldMap.metersToPixels(QPointF(x, -mapHeight/2));
        QPointF p2 = worldMap.metersToPixels(QPointF(x, mapHeight/2));
        scene.addLine(p1.x(), p1.y(), p2.x(), p2.y(), QPen(Qt::gray, 0.5));
    }
    for (int y = -static_cast<int>(mapHeight/2); y <= mapHeight/2; y += 5) {
        QPointF p1 = worldMap.metersToPixels(QPointF(-mapWidth/2, y));
        QPointF p2 = worldMap.metersToPixels(QPointF(mapWidth/2, y));
        scene.addLine(p1.x(), p1.y(), p2.x(), p2.y(), QPen(Qt::gray, 0.5));
    }
    
    // 添加障碍物
    worldMap.addObstacle(QPointF(10, 10), 2.0);
    worldMap.addObstacle(QPointF(-15, 20), 3.0);
    worldMap.addObstacle(QPointF(20, -15), 1.5);
    
    // 创建车辆参数
    Vehicle::Parameters params;
    params.mass = 1500;          // kg
    params.lf = 1.5;             // 前轴到质心距离（米）
    params.lr = 1.5;             // 后轴到质心距离（米）
    params.iz = 3000;            // 转动惯量（kg·m²）
    params.cf = 80000;           // 前轮侧偏刚度（N/rad）
    params.cr = 80000;           // 后轮侧偏刚度（N/rad）
    params.max_steer = M_PI/4;   // 最大转向角（45度）
    params.max_accel = 3.0;      // 最大加速度（m/s²）
    params.max_decel = 8.0;      // 最大减速度（m/s²）

    // 创建车辆
    Vehicle* vehicle = new Vehicle(&worldMap, params);
    vehicle->setstate(QPointF(0, 0), 5.0, 0, 0, 0); // 初始状态
    vehicle->setSize(QSizeF(2.0, 4.0)); // (车宽, 车长) 单位：米
    vehicle->setColor(Qt::blue);
    vehicle->setSensorsVisible(true);
    worldMap.addVehicle(vehicle);
    
    // 模拟控制
    QTimer controlTimer;
    QObject::connect(&controlTimer, &QTimer::timeout, [&]() {
        // double randomValue = QRandomGenerator::global()->generateDouble() * 1.5 - 0.5;
        // double throttle = qBound(0.0, randomValue, 1.0);

        // double random_value = QRandomGenerator::global()->generateDouble() - 0.5;
        // double steer = qBound(-1.0, random_value, 1.0);
        // vehicle->setControlInput(throttle, steer);
        
        auto obstacles = vehicle->getObstaclesInRange(10.0);
        if (!obstacles.isEmpty()) {
            qDebug() << "Detected" << obstacles.size() << "obstacles within 10m";
        }
    });
    controlTimer.start(200); // 5Hz控制更新
    
    // 设置视图范围（留出一些边距）
    qreal margin = 10; // 边距（米）
    QPointF bottomLeft = worldMap.metersToPixels(QPointF(-mapWidth/2 - margin, -mapHeight/2 - margin));
    QPointF topRight = worldMap.metersToPixels(QPointF(mapWidth/2 + margin, mapHeight/2 + margin));
    QRectF sceneRect(bottomLeft, topRight);
    
    // 根据屏幕大小调整视图
    qreal scale = qMin(
        static_cast<qreal>(screenWidth - 100) / sceneRect.width(),
        static_cast<qreal>(screenHeight - 100) / sceneRect.height()
    );
    
    view.setSceneRect(sceneRect);
    view.resize(screenWidth * 0.8, screenHeight * 0.8); // 窗口大小为屏幕的80%
    view.scale(scale, scale); // 初始缩放以适应屏幕
    
    view.show();
    return app.exec();
}