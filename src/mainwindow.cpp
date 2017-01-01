#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QProgressDialog>
#include <QPainter>

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow) {
    ui->setupUi(this);
    ui->LineImg1->setReadOnly(true);
    ui->LineImg2->setReadOnly(true);
    this->setWindowFlags(windowFlags() & ~Qt::WindowMaximizeButtonHint);
    this->setFixedSize(this->width(), this->height());
    form = new Form();
    form->setWindowFlags(windowFlags() & ~Qt::WindowMaximizeButtonHint);
    path_img1 = tr("");
    path_img2 = tr("");
    path_img_expand = tr("");
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_ButtonImg1_clicked() {
    QFileDialog *fileDialog = new QFileDialog(this, tr("打开图片"), tr("."), tr("Image Files(*.jpg *.png *.tiff)"));
    fileDialog->setWindowIcon(QIcon(":icon/res/icon.png"));
    if (fileDialog->exec() == QDialog::Accepted) {
        path_img1 = fileDialog->selectedFiles()[0];
        ui->LineImg1->setText(path_img1);
    }
}

void MainWindow::on_ButtonImg2_clicked() {
    QFileDialog *fileDialog = new QFileDialog(this, tr("打开图片"), tr("."), tr("Image Files(*.jpg *.png *.tiff)"));
    fileDialog->setWindowIcon(QIcon(":icon/res/icon.png"));
    if (fileDialog->exec() == QDialog::Accepted) {
        path_img2 = fileDialog->selectedFiles()[0];
        ui->LineImg2->setText(path_img2);
    }
}

void MainWindow::on_ButtonRun_clicked() {
    if (path_img1 == tr("") || path_img2 == tr("")) {
        QMessageBox::warning(this, tr("错误"), tr("有一张或多张图像没有选择！"), QMessageBox::Ok);
        return;
    }
    QProgressDialog *pd = new QProgressDialog(this);
    pd->setWindowIcon(QIcon(tr(":icon/res/icon.png")));
    pd->setWindowTitle(tr("正在运行"));
    pd->setModal(true);
    pd->setRange(0, 100);
    pd->show();

    pd->setLabelText(tr("正在读取第一张图像..."));
    pd->setValue(0);
    img1.load(path_img1);

    pd->setLabelText(tr("正在读取第二张图像..."));
    pd->setValue(5);
    img2.load(path_img2);

    pd->setLabelText(tr("正在对第一张图像寻找SIFT特征点..."));
    pd->setValue(40);
//    SIFT sift1(img1.width(), img1.height(), 3);
//    uchar ***tmp1 = new uchar **[img1.height()];
//    for (int i = 0; i != img1.height(); ++i) {
//        tmp1[i] = new uchar *[img1.width()];
//        for (int j = 0; j != img1.width(); ++j) {
//            tmp1[i][j] = new uchar[3];
//            tmp1[i][j][0] = img1.bits()[i * img1.width() * 4 + j * 4 + 2];
//            tmp1[i][j][1] = img1.bits()[i * img1.width() * 4 + j * 4 + 1];
//            tmp1[i][j][2] = img1.bits()[i * img1.width() * 4 + j * 4];
//        }
//    }
//    sift1.SetImg(tmp1);
//    sift1.run_sift();
//
//    pd->setLabelText(tr("正在对第二张图像寻找SIFT特征点..."));
//    pd->setValue(30);
//    SIFT sift2(img2.width(), img2.height(), 3);
//    uchar ***tmp2 = new uchar **[img2.height()];
//    for (int i = 0; i != img2.height(); ++i) {
//        tmp2[i] = new uchar *[img2.width()];
//        for (int j = 0; j != img2.width(); ++j) {
//            tmp2[i][j] = new uchar[3];
//            tmp2[i][j][0] = img2.bits()[i * img2.width() * 4 + j * 4 + 2];
//            tmp2[i][j][1] = img2.bits()[i * img2.width() * 4 + j * 4 + 1];
//            tmp2[i][j][2] = img2.bits()[i * img2.width() * 4 + j * 4];
//        }
//    }
//    sift2.SetImg(tmp2);
//    sift2.run_sift();
//
//    pd->setLabelText(tr("正在建立第一张图像的K-D树..."));
//    pd->setValue(50);
//    kdtree tree1;
//    tree1.setFinalKeyPoints(sift1.GetFinalKeypoint());
//    tree1.createKdtree();
//
//    pd->setLabelText(tr("正在建立第二张图像的K-D树..."));
//    pd->setValue(65);
//    kdtree tree2;
//    tree2.setFinalKeyPoints(sift2.GetFinalKeypoint());
//    tree2.createKdtree();
//
//    pd->setLabelText(tr("正在匹配特征点..."));
//    pd->setValue(80);
//    match mth;
//    mth.setKdtree(tree1, tree2);
//    mth.run_match();
//    match_line *mtl = mth.GetMatchLines();
//    pd->setLabelText(tr("完成！"));
//    pd->setValue(100);

    form->setImage(img1, img2);
    form->show();
    form->setFixedSize(form->width(), form->height());
    //form->drawMatches(mtl, mth.GetMatchLinesNum());
}

void MainWindow::on_ButtonQuit_clicked() {
    this->close();
}

void MainWindow::on_ButtonSave_clicked() {
    if (path_img1 == tr("") || path_img2 == tr("")) {
        QMessageBox::warning(this, tr("错误"), tr("有一张或多张图像没有选择！"), QMessageBox::Ok);
        return;
    }
    QFileDialog *fileDialog = new QFileDialog(this, tr("保存图片"), tr("."), tr("Image Files(*.jpg *.png *.tiff)"));
    fileDialog->setWindowIcon(QIcon(":icon/res/icon.png"));
    if (fileDialog->exec() == QDialog::Accepted) {
        path_img_expand = fileDialog->selectedFiles()[0];
    } else {
        return;
    }
    img_expand = new QImage((img1.width() + img2.width()), max(img1.height(), img2.height()), img1.format());
    for (int i = 0; i != img_expand->height(); ++i) {
        for (int j = 0; j != img_expand->width(); ++j) {
            img_expand->setPixel(j, i, qRgb(0, 0, 0));
        }
    }
    for (int i = 0; i != img1.width(); ++i) {
        for (int j = 0; j != img1.height(); ++j) {
            img_expand->setPixel(i, j, img1.pixel(i, j));
        }
    }
    for (int i = 0; i != img2.width(); ++i) {
        for (int j = 0; j != img2.height(); ++j) {
            img_expand->setPixel(i + img1.width(), j, img2.pixel(i, j));
        }
    }
    img_expand->save(path_img_expand);
}
