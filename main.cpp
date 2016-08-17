#include "mainwindow.h"
#include <QApplication>

// thread control
#include <QReadWriteLock>
QReadWriteLock lock_sv;
QReadWriteLock lock_sv_data;
QReadWriteLock lock_sv_object;
QReadWriteLock lock_sv_mouse;
QReadWriteLock lock_lrf;
QReadWriteLock lock_radar;

QReadWriteLock lock_f_sv;
QReadWriteLock lock_f_topview;
QReadWriteLock lock_data_fused;
QReadWriteLock lock_ot_fused;
QReadWriteLock lock_a_star;

#include <QDir>
QDir project_path;
#include "recording/recording.h"
recording re(IMG_H, IMG_W);

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
