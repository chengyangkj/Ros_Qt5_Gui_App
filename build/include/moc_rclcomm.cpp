/****************************************************************************
** Meta object code from reading C++ file 'rclcomm.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/rclcomm.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'rclcomm.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rclcomm_t {
    QByteArrayData data[13];
    char stringdata0[146];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rclcomm_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rclcomm_t qt_meta_stringdata_rclcomm = {
    {
QT_MOC_LITERAL(0, 0, 7), // "rclcomm"
QT_MOC_LITERAL(1, 8, 13), // "emitTopicData"
QT_MOC_LITERAL(2, 22, 0), // ""
QT_MOC_LITERAL(3, 23, 13), // "emitUpdateMap"
QT_MOC_LITERAL(4, 37, 3), // "img"
QT_MOC_LITERAL(5, 41, 19), // "emitUpdateRobotPose"
QT_MOC_LITERAL(6, 61, 9), // "RobotPose"
QT_MOC_LITERAL(7, 71, 4), // "pose"
QT_MOC_LITERAL(8, 76, 20), // "emitUpdateLaserPoint"
QT_MOC_LITERAL(9, 97, 6), // "points"
QT_MOC_LITERAL(10, 104, 14), // "emitUpdatePath"
QT_MOC_LITERAL(11, 119, 11), // "pubInitPose"
QT_MOC_LITERAL(12, 131, 14) // "pubNavGoalPose"

    },
    "rclcomm\0emitTopicData\0\0emitUpdateMap\0"
    "img\0emitUpdateRobotPose\0RobotPose\0"
    "pose\0emitUpdateLaserPoint\0points\0"
    "emitUpdatePath\0pubInitPose\0pubNavGoalPose"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rclcomm[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x06 /* Public */,
       3,    1,   52,    2, 0x06 /* Public */,
       5,    1,   55,    2, 0x06 /* Public */,
       8,    1,   58,    2, 0x06 /* Public */,
      10,    1,   61,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      11,    2,   64,    2, 0x0a /* Public */,
      12,    2,   69,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QImage,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, QMetaType::QPolygonF,    9,
    QMetaType::Void, QMetaType::QPolygonF,    9,

 // slots: parameters
    QMetaType::Void, QMetaType::QPointF, QMetaType::QPointF,    2,    2,
    QMetaType::Void, QMetaType::QPointF, QMetaType::QPointF,    2,    2,

       0        // eod
};

void rclcomm::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<rclcomm *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->emitTopicData((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->emitUpdateMap((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 2: _t->emitUpdateRobotPose((*reinterpret_cast< RobotPose(*)>(_a[1]))); break;
        case 3: _t->emitUpdateLaserPoint((*reinterpret_cast< QPolygonF(*)>(_a[1]))); break;
        case 4: _t->emitUpdatePath((*reinterpret_cast< QPolygonF(*)>(_a[1]))); break;
        case 5: _t->pubInitPose((*reinterpret_cast< QPointF(*)>(_a[1])),(*reinterpret_cast< QPointF(*)>(_a[2]))); break;
        case 6: _t->pubNavGoalPose((*reinterpret_cast< QPointF(*)>(_a[1])),(*reinterpret_cast< QPointF(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (rclcomm::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&rclcomm::emitTopicData)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (rclcomm::*)(QImage );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&rclcomm::emitUpdateMap)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (rclcomm::*)(RobotPose );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&rclcomm::emitUpdateRobotPose)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (rclcomm::*)(QPolygonF );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&rclcomm::emitUpdateLaserPoint)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (rclcomm::*)(QPolygonF );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&rclcomm::emitUpdatePath)) {
                *result = 4;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rclcomm::staticMetaObject = { {
    &QThread::staticMetaObject,
    qt_meta_stringdata_rclcomm.data,
    qt_meta_data_rclcomm,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rclcomm::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rclcomm::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rclcomm.stringdata0))
        return static_cast<void*>(this);
    return QThread::qt_metacast(_clname);
}

int rclcomm::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void rclcomm::emitTopicData(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void rclcomm::emitUpdateMap(QImage _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void rclcomm::emitUpdateRobotPose(RobotPose _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void rclcomm::emitUpdateLaserPoint(QPolygonF _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void rclcomm::emitUpdatePath(QPolygonF _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
