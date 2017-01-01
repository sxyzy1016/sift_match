#ifndef FORM_H
#define FORM_H

#include <QWidget>
#include <QImage>
#include "match.h"

namespace Ui {
    class Form;
}

class Form : public QWidget {
Q_OBJECT

public:
    explicit Form(QWidget *parent = 0);

    ~Form();

private:
    Ui::Form *ui;

public:
    void setImage(QImage _img1, QImage _img2);

    void drawMatches(match_line *_mtl, int _mtn);
};

#endif // FORM_H
