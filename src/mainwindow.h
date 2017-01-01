#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QImage>
#include "form.h"
#include "SIFT.h"
#include "kdtree.h"
#include "match.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_ButtonImg1_clicked();

    void on_ButtonImg2_clicked();

    void on_ButtonRun_clicked();

    void on_ButtonQuit_clicked();

    void on_ButtonSave_clicked();

private:
    Ui::MainWindow *ui;

private:
    QString path_img1,path_img2,path_img_expand;
    QImage img1,img2,*img_expand;
    Form* form;
};

#endif // MAINWINDOW_H
