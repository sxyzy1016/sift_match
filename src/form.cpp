#include "form.h"
#include "ui_form.h"
#include <QPainter>

Form::Form(QWidget *parent) :
        QWidget(parent),
        ui(new Ui::Form) {
    ui->setupUi(this);
}

Form::~Form() {
    delete ui;
}

void Form::setImage(QImage _img1, QImage _img2) {
    ui->LabelShowImg1->setPixmap(QPixmap::fromImage(_img1));
    ui->LabelShowImg2->setPixmap(QPixmap::fromImage(_img2));
}

void Form::drawMatches(match_line *_mtl, int _mtn) {
    QPainter painter(this);
    QPen pen;
    pen.setStyle(Qt::DashLine);
    pen.setColor(Qt::yellow);
    painter.setPen(pen);
    for (int i = 0; i != _mtn; ++i) {
        painter.drawLine(_mtl[i].p1.img_x, _mtl[i].p1.img_y, _mtl[i].p2.img_x, _mtl[i].p2.img_y);
    }
}
