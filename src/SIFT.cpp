//
// Created by SXY on 2016-12-29.
//
#include <cmath>
#include <cstdio>
#include <cstring>
#include "SIFT.h"

using namespace std;

int SIFT::rgb2gray(uchar ***src, uchar **dst, double _width, double _height) {
    int i, j;
    for (i = 0; i != _height; ++i) {
        for (j = 0; j != _width; ++j) {
            dst[i][j] = (uchar) (((int) (src[i][j][0]) * 76 + (int) (src[i][j][1]) * 150 + (int) (src[i][j][2]) * 30)
                    >> 8);
        }
    }
    return 0;
}

int SIFT::gaussianBlur(uchar **src, uchar **dst, double _width, double _height, double sigma) {
    int size = GAUSSIAN_MODAL_SIZE % 2 == 0 ? (GAUSSIAN_MODAL_SIZE + 1) : GAUSSIAN_MODAL_SIZE;
    double gaussian_modal[GAUSSIAN_MODAL_SIZE][GAUSSIAN_MODAL_SIZE];
    int i, j;
    for (i = (-1) * (size - 1) / 2; i != (size + 1) / 2; ++i) {
        for (j = (-1) * (size - 1) / 2; j != (size + 1) / 2; ++j) {
            long double t;
            t = i * i + j * j;
            t /= (2 * sigma * sigma);
            t = exp(t);
            t /= (2 * PI * sigma);
            gaussian_modal[i + (size - 1) / 2][j + (size - 1) / 2] = (double) t;
        }
    }
    int width = (int) _width, height = (int) _height;
    for (i = 0; i != height; ++i) {
        for (j = 0; j != width; ++j) {
            if ((i < ((size - 1) / 2) || i > (height - (size + 1) / 2)) ||
                (j < ((size - 1) / 2) || j > (width - (size + 1) / 2))) {
                dst[i][j] = src[i][j];
            } else {
                int m, n;
                double total = 0.0;
                for (m = (-1) * (size - 1) / 2; m != (size + 1) / 2; ++m) {
                    for (n = (-1) * (size - 1) / 2; n != (size + 1) / 2; ++n) {
                        total += (double) (int) src[i + m][j + n] *
                                 gaussian_modal[m + (size - 1) / 2][n + (size - 1) / 2];
                    }
                }
                dst[i][j] = (uchar) (int) (total < 0 ? 0 : total > 255 ? 255 : total);
            }
        }
    }
    return 0;
}

int SIFT::buildScaleSpace(uchar **src, scaleSpace &dst, double _width, double _height, double _lry_num) {
    if (_lry_num < 6) _lry_num = 6;
    double s = _lry_num - 3;
    double k_s = (double) pow(2.0l, (long double) 1 / s);
    int width = (int) _width, height = (int) _height;
    dst.lry_num = (int) _lry_num;
    dst.width = width;
    dst.height = height;
    for (int m = 0; m != dst.lry_num; ++m) {
        if (m == 0) {
            gaussianBlur(src, dst.scale[m], width, height, SIFT_SIGMA_INITIAL);
        } else {
            gaussianBlur(src, dst.scale[m], width, height,
                         SIFT_SIGMA * (double) pow((long double) k_s, (long double) m + 1));
        }
    }
    return 0;
}

int SIFT::buildLoGPyramid(uchar **src, Pyramid &dst, double _width, double _height, double _octave) {
    _octave = _octave < (log(min(width, height)) / log(2.0) - 2) ? _octave : (log(min(width, height)) / log(2.0) - 2);
    dst.octave = (int) _octave;
    int width = (int) _width, height = (int) _height;
    int i;
    for (i = 0; i != dst.octave; ++i) {
        dst.layers[i].width = (int) (width / pow(2.0d, (double) i));
        dst.layers[i].height = (int) (height / pow(2.0d, (double) i));
        dst.layers[i].lry_num = DEFAULT_SCALE_LAYER_NUMBER;
        uchar **src_use;
        src_use = new uchar *[dst.layers[i].height];
        int m, n, r;
        for (m = 0; m != dst.layers[i].height; ++m) {
            src_use[m] = new uchar[dst.layers[i].width];
        }
        for (n = 0; n != dst.layers[i].height; ++n) {
            for (r = 0; r != dst.layers[i].width; ++r) {
                src_use[n][r] = src[(int) (n * pow(2.0d, (double) i))][(int) (r * pow(2.0d, (double) i))];
            }
        }
        buildScaleSpace(src_use, dst.layers[i], dst.layers[i].width, dst.layers[i].height, dst.layers[i].lry_num);
    }
    return 0;
}

int SIFT::buildDoGPyramid(Pyramid src, Pyramid &dst) {
    int i, j;
    dst.octave = src.octave;
    for (i = 0; i != src.octave; ++i) {
        dst.layers[i].lry_num = src.layers[i].lry_num - 1;
        dst.layers[i].width = src.layers[i].width;
        dst.layers[i].height = src.layers[i].height;
        for (j = 1; j != src.layers[i].lry_num; ++j) {
            int n, r;
            for (n = 0; n != src.layers[i].height; ++n) {
                for (r = 0; r != src.layers[i].width; ++r) {
                    dst.layers[i].scale[j][n][r] = (uchar) ((int) src.layers[i].scale[j][n][r] -
                                                            (int) src.layers[i].scale[j - 1][n][r]);
                }
            }
        }
    }
    return 0;
}

int SIFT::findScaleSpaceExtreme(scaleSpace src, Point *keypoint, int _scale_num) {
    keypoint->next = NULL;
    Point *p = keypoint;
    int i;
    for (i = 1; i != src.lry_num - 1; ++i) {
        int n, r;
        for (n = 1; n != src.height - 1; ++n) {
            for (r = 1; r != src.width - 1; ++r) {
                int value = (int) src.scale[i][n][r];
                if ((value > (int) src.scale[i][n - 1][r - 1] && value > (int) src.scale[i][n - 1][r] &&
                     value > (int) src.scale[i][n][r + 1] &&
                     value > (int) src.scale[i][n][r - 1] && value > (int) src.scale[i][n][r + 1] &&
                     value > (int) src.scale[i][n + 1][r - 1] &&
                     value > (int) src.scale[i][n + 1][r] && value > (int) src.scale[i][n + 1][r + 1] &&
                     value > (int) src.scale[i - 1][n - 1][r - 1] &&
                     value > (int) src.scale[i - 1][n - 1][r] && value > (int) src.scale[i - 1][n - 1][r + 1] &&
                     value > (int) src.scale[i - 1][n][r - 1] &&
                     value > (int) src.scale[i - 1][n][r] && value > (int) src.scale[i - 1][n][r + 1] &&
                     value > (int) src.scale[i - 1][n + 1][r - 1] &&
                     value > (int) src.scale[i - 1][n + 1][r] && value > (int) src.scale[i - 1][n + 1][r + 1] &&
                     value > (int) src.scale[i + 1][n - 1][r - 1] &&
                     value > (int) src.scale[i + 1][n - 1][r] && value > (int) src.scale[i + 1][n - 1][r + 1] &&
                     value > (int) src.scale[i + 1][n][r - 1] &&
                     value > (int) src.scale[i + 1][n][r] && value > (int) src.scale[i + 1][n][r + 1] &&
                     value > (int) src.scale[i + 1][n + 1][r - 1] &&
                     value > (int) src.scale[i + 1][n + 1][r] && value > (int) src.scale[i + 1][n + 1][r + 1]) ||
                    (value < (int) src.scale[i][n - 1][r - 1] && value < (int) src.scale[i][n - 1][r] &&
                     value < (int) src.scale[i][n][r + 1] &&
                     value < (int) src.scale[i][n][r - 1] && value < (int) src.scale[i][n][r + 1] &&
                     value < (int) src.scale[i][n + 1][r - 1] &&
                     value < (int) src.scale[i][n + 1][r] && value < (int) src.scale[i][n + 1][r + 1] &&
                     value < (int) src.scale[i - 1][n - 1][r - 1] &&
                     value < (int) src.scale[i - 1][n - 1][r] && value < (int) src.scale[i - 1][n - 1][r + 1] &&
                     value < (int) src.scale[i - 1][n][r - 1] &&
                     value < (int) src.scale[i - 1][n][r] && value < (int) src.scale[i - 1][n][r + 1] &&
                     value < (int) src.scale[i - 1][n + 1][r - 1] &&
                     value < (int) src.scale[i - 1][n + 1][r] && value < (int) src.scale[i - 1][n + 1][r + 1] &&
                     value < (int) src.scale[i + 1][n - 1][r - 1] &&
                     value < (int) src.scale[i + 1][n - 1][r] && value < (int) src.scale[i + 1][n - 1][r + 1] &&
                     value < (int) src.scale[i + 1][n][r - 1] &&
                     value < (int) src.scale[i + 1][n][r] && value < (int) src.scale[i + 1][n][r + 1] &&
                     value < (int) src.scale[i + 1][n + 1][r - 1] &&
                     value < (int) src.scale[i + 1][n + 1][r] && value < (int) src.scale[i + 1][n + 1][r + 1])) {
                    p->next = new Point;
                    p = p->next;
                    p->scale_x = r;
                    p->scale_y = n;
                    p->img_x = r * pow(2.0d, (double) i);
                    p->img_y = n * pow(2.0d, (double) i);
                    p->lry_num = i;
                    p->scale_num = _scale_num;
                    p->value = (uchar) value;
                    p->next = NULL;
                }
            }
        }
    }
    return 0;
}

int SIFT::findDoGExtreme(Pyramid src, Point *keypoint) {
    int i;
    keypoint->next = NULL;
    Point *p = keypoint;
    for (i = 0; i != src.octave; ++i) {
        Point *kp_use = new Point;
        findScaleSpaceExtreme(src.layers[i], kp_use, i);
        p->next = kp_use->next;
        kp_use->next = NULL;
        delete kp_use;
        while (p->next) {
            p = p->next;
        }
    }
    return 0;
}

int SIFT::findExactKeypoint(Pyramid LoG, Point *src, Point *dst) {
    Point *ps = src;
    Point *pd = dst;
    while (ps->next) {
        ps = ps->next;
        double dDdX[3];
        dDdX[0] = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x + 1] -
                            (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x - 1]) / 2.0;
        dDdX[1] = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y + 1][ps->scale_x] -
                            (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y + 1][ps->scale_x]) / 2.0;
        dDdX[2] = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num + 1][ps->scale_y][ps->scale_x] -
                            (int) LoG.layers[ps->scale_num].scale[ps->lry_num - 1][ps->scale_y][ps->scale_x]) / 2.0;
        double d2DdX2[3][3];
        double dxx, dyy, dss, dxy, dxs, dys;
        dxx = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x - 1] +
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x + 1] -
                        2 * (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x]);
        dyy = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y - 1][ps->scale_x] +
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y + 1][ps->scale_x] -
                        2 * (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x]);
        dss = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num - 1][ps->scale_y][ps->scale_x] +
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num + 1][ps->scale_y][ps->scale_x] -
                        2 * (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x]);
        dxy = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y + 1][ps->scale_x + 1] +
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y - 1][ps->scale_x - 1] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y + 1][ps->scale_x - 1] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y - 1][ps->scale_x + 1]) / 4.0;
        dxs = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num + 1][ps->scale_y][ps->scale_x + 1] +
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num - 1][ps->scale_y][ps->scale_x - 1] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num + 1][ps->scale_y][ps->scale_x - 1] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num - 1][ps->scale_y][ps->scale_x + 1]) / 4.0;
        dys = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num + 1][ps->scale_y + 1][ps->scale_x] +
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num - 1][ps->scale_y - 1][ps->scale_x] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num + 1][ps->scale_y - 1][ps->scale_x] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num - 1][ps->scale_y + 1][ps->scale_x]) / 4.0;
        d2DdX2[0][0] = dxx;
        d2DdX2[0][1] = dxy;
        d2DdX2[0][2] = dxs;
        d2DdX2[1][0] = dxy;
        d2DdX2[1][1] = dyy;
        d2DdX2[1][2] = dys;
        d2DdX2[2][0] = dxs;
        d2DdX2[2][1] = dys;
        d2DdX2[2][2] = dss;
        double d2DdX2_mod = d2DdX2[0][0] * d2DdX2[1][1] * d2DdX2[2][2] + d2DdX2[1][0] * d2DdX2[2][1] * d2DdX2[0][2] +
                            d2DdX2[0][1] * d2DdX2[1][2] * d2DdX2[2][0] -
                            d2DdX2[2][0] * d2DdX2[1][1] * d2DdX2[0][2] - d2DdX2[0][1] * d2DdX2[1][0] * d2DdX2[2][2] -
                            d2DdX2[1][2] * d2DdX2[2][1] * d2DdX2[0][0];
        double d2DdX2_inv[3][3];
        d2DdX2_inv[0][0] = (d2DdX2[1][1] * d2DdX2[2][2] - d2DdX2[1][2] * d2DdX2[2][1]) / d2DdX2_mod;
        d2DdX2_inv[0][1] = (d2DdX2[1][0] * d2DdX2[2][2] - d2DdX2[1][2] * d2DdX2[2][0]) / d2DdX2_mod;
        d2DdX2_inv[0][2] = (d2DdX2[1][0] * d2DdX2[2][1] - d2DdX2[1][1] * d2DdX2[2][0]) / d2DdX2_mod;
        d2DdX2_inv[1][0] = (d2DdX2[0][1] * d2DdX2[2][2] - d2DdX2[0][2] * d2DdX2[2][1]) / d2DdX2_mod;
        d2DdX2_inv[1][1] = (d2DdX2[0][0] * d2DdX2[2][2] - d2DdX2[0][2] * d2DdX2[2][0]) / d2DdX2_mod;
        d2DdX2_inv[1][2] = (d2DdX2[0][0] * d2DdX2[2][1] - d2DdX2[2][0] * d2DdX2[0][1]) / d2DdX2_mod;
        d2DdX2_inv[2][0] = (d2DdX2[0][1] * d2DdX2[1][2] - d2DdX2[1][1] * d2DdX2[0][2]) / d2DdX2_mod;
        d2DdX2_inv[2][1] = (d2DdX2[0][0] * d2DdX2[1][2] - d2DdX2[1][0] * d2DdX2[0][2]) / d2DdX2_mod;
        d2DdX2_inv[2][2] = (d2DdX2[0][0] * d2DdX2[1][1] - d2DdX2[1][0] * d2DdX2[0][1]) / d2DdX2_mod;
        double off_X[3];
        off_X[0] = (-1) * (d2DdX2_inv[0][0] * dDdX[0] + d2DdX2[0][1] * dDdX[1] + d2DdX2[0][2] * dDdX[2]);
        off_X[1] = (-1) * (d2DdX2_inv[1][0] * dDdX[0] + d2DdX2[1][1] * dDdX[1] + d2DdX2[1][2] * dDdX[2]);
        off_X[2] = (-1) * (d2DdX2_inv[2][0] * dDdX[0] + d2DdX2[2][1] * dDdX[1] + d2DdX2[2][2] * dDdX[2]);
        if (abs(off_X[0]) < 0.5 || abs(off_X[1]) < 0.5 || abs(off_X[2]) < 0.5) {
            ps->exact_x = ps->scale_x;
            ps->exact_y = ps->scale_y;
            ps->exact_lry = ps->lry_num;
        } else {
            ps->exact_x = ps->scale_x + round(off_X[0]);
            ps->exact_y = ps->scale_y + round(off_X[1]);
            ps->exact_lry = ps->lry_num + round(off_X[2]);
        }
        if (ps->exact_lry < 1 || ps->exact_lry > LoG.layers[ps->scale_num].lry_num ||
            ps->exact_x < SIFT_KEYPOINT_BORDER ||
            ps->exact_y < SIFT_KEYPOINT_BORDER ||
            ps->exact_x >= (LoG.layers[ps->scale_num].width - SIFT_KEYPOINT_BORDER) ||
            ps->exact_y >= (LoG.layers[ps->scale_num].height - SIFT_KEYPOINT_BORDER)) {
            continue;
        } else {
            pd->next = new Point;
            pd = pd->next;
            pd->next = NULL;
            pd->exact_x = ps->exact_x;
            pd->exact_y = ps->exact_y;
            pd->exact_lry = ps->exact_lry;
            pd->img_x = ps->img_x;
            pd->img_y = ps->img_y;
            pd->scale_x = ps->scale_x;
            pd->scale_y = ps->scale_y;
            pd->scale_num = ps->scale_num;
            pd->lry_num = ps->lry_num;
            pd->value = ps->value;
        }
    }
    return 0;
}

int SIFT::checkContrastKeypoint(Pyramid LoG, Point *src, Point *dst) {
    Point *ps = src;
    Point *pd = dst;
    while (ps->next) {
        ps = ps->next;
        double dDdX[3];
        dDdX[0] = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x + 1] -
                            (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x - 1]) / 2.0;
        dDdX[1] = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y + 1][ps->scale_x] -
                            (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y + 1][ps->scale_x]) / 2.0;
        dDdX[2] = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num + 1][ps->scale_y][ps->scale_x] -
                            (int) LoG.layers[ps->scale_num].scale[ps->lry_num - 1][ps->scale_y][ps->scale_x]) / 2.0;
        double d2DdX2[3][3];
        double dxx, dyy, dss, dxy, dxs, dys;
        dxx = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x - 1] +
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x + 1] -
                        2 * (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x]);
        dyy = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y - 1][ps->scale_x] +
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y + 1][ps->scale_x] -
                        2 * (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x]);
        dss = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num - 1][ps->scale_y][ps->scale_x] +
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num + 1][ps->scale_y][ps->scale_x] -
                        2 * (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x]);
        dxy = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y + 1][ps->scale_x + 1] +
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y - 1][ps->scale_x - 1] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y + 1][ps->scale_x - 1] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y - 1][ps->scale_x + 1]) / 4.0;
        dxs = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num + 1][ps->scale_y][ps->scale_x + 1] +
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num - 1][ps->scale_y][ps->scale_x - 1] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num + 1][ps->scale_y][ps->scale_x - 1] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num - 1][ps->scale_y][ps->scale_x + 1]) / 4.0;
        dys = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num + 1][ps->scale_y + 1][ps->scale_x] +
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num - 1][ps->scale_y - 1][ps->scale_x] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num + 1][ps->scale_y - 1][ps->scale_x] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num - 1][ps->scale_y + 1][ps->scale_x]) / 4.0;
        d2DdX2[0][0] = dxx;
        d2DdX2[0][1] = dxy;
        d2DdX2[0][2] = dxs;
        d2DdX2[1][0] = dxy;
        d2DdX2[1][1] = dyy;
        d2DdX2[1][2] = dys;
        d2DdX2[2][0] = dxs;
        d2DdX2[2][1] = dys;
        d2DdX2[2][2] = dss;
        double d2DdX2_mod = d2DdX2[0][0] * d2DdX2[1][1] * d2DdX2[2][2] + d2DdX2[1][0] * d2DdX2[2][1] * d2DdX2[0][2] +
                            d2DdX2[0][1] * d2DdX2[1][2] * d2DdX2[2][0] -
                            d2DdX2[2][0] * d2DdX2[1][1] * d2DdX2[0][2] - d2DdX2[0][1] * d2DdX2[1][0] * d2DdX2[2][2] -
                            d2DdX2[1][2] * d2DdX2[2][1] * d2DdX2[0][0];
        double d2DdX2_inv[3][3];
        d2DdX2_inv[0][0] = (d2DdX2[1][1] * d2DdX2[2][2] - d2DdX2[1][2] * d2DdX2[2][1]) / d2DdX2_mod;
        d2DdX2_inv[0][1] = (d2DdX2[1][0] * d2DdX2[2][2] - d2DdX2[1][2] * d2DdX2[2][0]) / d2DdX2_mod;
        d2DdX2_inv[0][2] = (d2DdX2[1][0] * d2DdX2[2][1] - d2DdX2[1][1] * d2DdX2[2][0]) / d2DdX2_mod;
        d2DdX2_inv[1][0] = (d2DdX2[0][1] * d2DdX2[2][2] - d2DdX2[0][2] * d2DdX2[2][1]) / d2DdX2_mod;
        d2DdX2_inv[1][1] = (d2DdX2[0][0] * d2DdX2[2][2] - d2DdX2[0][2] * d2DdX2[2][0]) / d2DdX2_mod;
        d2DdX2_inv[1][2] = (d2DdX2[0][0] * d2DdX2[2][1] - d2DdX2[2][0] * d2DdX2[0][1]) / d2DdX2_mod;
        d2DdX2_inv[2][0] = (d2DdX2[0][1] * d2DdX2[1][2] - d2DdX2[1][1] * d2DdX2[0][2]) / d2DdX2_mod;
        d2DdX2_inv[2][1] = (d2DdX2[0][0] * d2DdX2[1][2] - d2DdX2[1][0] * d2DdX2[0][2]) / d2DdX2_mod;
        d2DdX2_inv[2][2] = (d2DdX2[0][0] * d2DdX2[1][1] - d2DdX2[1][0] * d2DdX2[0][1]) / d2DdX2_mod;
        double off_X[3];
        off_X[0] = (-1) * (d2DdX2_inv[0][0] * dDdX[0] + d2DdX2[0][1] * dDdX[1] + d2DdX2[0][2] * dDdX[2]);
        off_X[1] = (-1) * (d2DdX2_inv[1][0] * dDdX[0] + d2DdX2[1][1] * dDdX[1] + d2DdX2[1][2] * dDdX[2]);
        off_X[2] = (-1) * (d2DdX2_inv[2][0] * dDdX[0] + d2DdX2[2][1] * dDdX[1] + d2DdX2[2][2] * dDdX[2]);
        double t = dDdX[0] * off_X[0] + dDdX[1] * off_X[1] + dDdX[2] * off_X[2];
        t *= 0.5;
        t += LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x];
        if (abs(t) < SIFT_CONTRAST_THETA) {
            continue;
        } else {
            pd->next = new Point;
            pd = pd->next;
            pd->next = NULL;
            pd->exact_x = ps->exact_x;
            pd->exact_y = ps->exact_y;
            pd->exact_lry = ps->exact_lry;
            pd->img_x = ps->img_x;
            pd->img_y = ps->img_y;
            pd->scale_x = ps->scale_x;
            pd->scale_y = ps->scale_y;
            pd->scale_num = ps->scale_num;
            pd->lry_num = ps->lry_num;
            pd->value = ps->value;
        }
    }
    return 0;
}

int SIFT::adjustExtremePoints(Pyramid LoG, Point *src, Point *dst) {
    Point *ps = src;
    Point *pd = dst;
    while (ps->next) {
        ps = ps->next;
        double dxx, dyy, dxy;
        dxx = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x + 1] +
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x - 1] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x] * 2);
        dyy = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y + 1][ps->scale_x] +
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y - 1][ps->scale_x] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y][ps->scale_x] * 2);
        dxy = (double) ((int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y + 1][ps->scale_x + 1] +
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y - 1][ps->scale_x - 1] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y + 1][ps->scale_x - 1] -
                        (int) LoG.layers[ps->scale_num].scale[ps->lry_num][ps->scale_y - 1][ps->scale_x + 1]) / 4.0;
        double tr, det;
        tr = dxx + dyy;
        det = dxx * dyy - dxy * dxy;
        if (det <= 0) {
            continue;
        } else {
            if (tr * tr / det < (SIFT_EDGE_R + 1.0) * (SIFT_EDGE_R + 1.0) / SIFT_EDGE_R) {
                pd->next = new Point;
                pd = pd->next;
                pd->next = NULL;
                pd->exact_x = ps->exact_x;
                pd->exact_y = ps->exact_y;
                pd->exact_lry = ps->exact_lry;
                pd->img_x = ps->img_x;
                pd->img_y = ps->img_y;
                pd->scale_x = ps->scale_x;
                pd->scale_y = ps->scale_y;
                pd->scale_num = ps->scale_num;
                pd->lry_num = ps->lry_num;
                pd->value = ps->value;
            } else {
                continue;
            }
        }
    }
    return 0;
}

int SIFT::buildFirstOrientation(Pyramid LoG, Point *src, finalKeyPoint *dst) {
    Point *ps = src;
    finalKeyPoint *pd = dst;
    while (ps->next) {
        ps = ps->next;
        int i, j;
        double hist[SIFT_HIST_BIN] = {0};
        double radius = round(SIFT_ORI_RADIUS * pow(2.0d, ps->exact_lry / (LoG.layers[ps->scale_num].lry_num - 3.0)));
        double sigma = SIFT_SIGMA * pow(2.0d, ps->exact_lry / (LoG.layers[ps->scale_num].lry_num - 3.0));
        double dx, dy, mod, ori;
        for (i = -1 * (int) radius; i != (int) radius + 1; ++i) {
            for (j = -1 * (int) radius; j != (int) radius + 1; ++j) {
                if (ps->exact_x > 0 && ps->exact_x < LoG.layers[ps->scale_num].width &&
                    ps->exact_y > 0 && ps->exact_y < LoG.layers[ps->scale_num].height) {
                    dx = LoG.layers[ps->scale_num].scale[(int) ps->exact_lry][(int) ps->exact_y][(int) ps->exact_x +
                                                                                                 1] -
                         LoG.layers[ps->scale_num].scale[(int) ps->exact_lry][(int) ps->exact_y][(int) ps->exact_x - 1];
                    dy = LoG.layers[ps->scale_num].scale[(int) ps->exact_lry][(int) ps->exact_y +
                                                                              1][(int) ps->exact_x] -
                         LoG.layers[ps->scale_num].scale[(int) ps->exact_lry][(int) ps->exact_y - 1][(int) ps->exact_x];
                    mod = sqrt(dx * dx + dy * dy);
                    ori = atan(dy / dx);
                    double weight, bin;
                    weight = exp(-1 * (i * i + j * j) / (2 * sigma * sigma)) / (2 * PI * sigma);
                    bin = round(SIFT_HIST_BIN * (ori + PI) / (2.0 * PI));
                    bin = (bin < SIFT_HIST_BIN) ? bin : 0;
                    hist[(int) bin] += weight * mod;
                }
            }
        }
        for (i = 0; i != HIST_SMOOTH_PASSES; ++i) {
            double prev, tmp, h0;
            h0 = hist[0];
            prev = hist[SIFT_HIST_BIN - 1];
            for (j = 0; j != SIFT_HIST_BIN; ++j) {
                tmp = hist[j];
                hist[j] = 0.25 * prev + 0.5 * hist[j] + 0.25 * ((j + 1) == SIFT_HIST_BIN ? h0 : hist[j + 1]);
                prev = tmp;
            }
        }
        double ori_max = hist[0];
        for (i = 0; i != SIFT_HIST_BIN; ++i) {
            ori_max = (ori_max > hist[i]) ? ori_max : hist[i];
        }
        bool jugg[SIFT_HIST_BIN] = {false};
        pd->next = new finalKeyPoint;
        pd = pd->next;
        pd->p = *ps;
        pd->p.next = NULL;
        memset(pd->oriens, 0, sizeof(pd->oriens));
        pd->first_ori_num = 0;
        for (i = 0; i != SIFT_HIST_BIN; ++i) {
            int l, r;
            l = (i == 0) ? (SIFT_HIST_BIN - 1) : (i - 1);
            r = (i + 1) % SIFT_HIST_BIN;
            if (hist[i] > hist[l] && hist[i] > hist[r] && hist[i] >= SIFT_ORI_PEAK_RAD * ori_max) {
                jugg[i] = true;
                ++(pd->first_ori_num);
            }
        }
        pd->first_ori = new Orientation[pd->first_ori_num];
        j = 0;
        for (i = 0; i != SIFT_HIST_BIN; ++i) {
            if (jugg[i]) {
                int l, r;
                l = (i == 0) ? (SIFT_HIST_BIN - 1) : (i - 1);
                r = (i + 1) % SIFT_HIST_BIN;
                double bin = i + 0.5 * (hist[l] - hist[r]) / (hist[l] - 2.0 * hist[i] + hist[r]);
                bin = (bin < 0) ? (SIFT_HIST_BIN + bin) : (bin >= SIFT_HIST_BIN) ? (bin - SIFT_HIST_BIN) : bin;
                pd->first_ori[j].theta = ((2.0 * PI * bin) / SIFT_HIST_BIN) - PI;
                pd->first_ori[j].weight = hist[i];
                ++j;
            }
        }
    }
    return 0;
}

int SIFT::buildKeyPointDescriptor(Pyramid LoG, finalKeyPoint *src, finalKeyPoint *dst) {
    finalKeyPoint *ps = src;
    finalKeyPoint *pd = dst;
    while (ps->next) {
        ps = ps->next;
        int i;
        pd->next = new finalKeyPoint;
        pd = pd->next;
        pd->p = ps->p;
        *pd->first_ori = *ps->first_ori;
        pd->first_ori_num = ps->first_ori_num;
        pd->next = NULL;
        memset(pd->oriens, 0, sizeof(pd->oriens));
        for (i = 0; i != KEYPOINT_DESCRIPTOR_RADIUS; ++i) {
            int n, m;
            m = (i - i % 4) / 4;
            n = i % 4;
            int p, q;
            for (p = 0; p != 4 * (m + 1); ++p) {
                for (q = 0; q != 4 * (n + 1); ++q) {
                    if (ps->p.exact_x > 0 && ps->p.exact_x < LoG.layers[ps->p.scale_num].width &&
                        ps->p.exact_y > 0 && ps->p.exact_y < LoG.layers[ps->p.scale_num].height) {
                        double dx, dy, mod, ori;
                        double radius = round(SIFT_ORI_RADIUS *
                                              pow(2.0d, ps->p.exact_lry / (LoG.layers[ps->p.scale_num].lry_num - 3.0)));
                        double sigma =
                                SIFT_SIGMA * pow(2.0d, ps->p.exact_lry / (LoG.layers[ps->p.scale_num].lry_num - 3.0));
                        dx = LoG.layers[ps->p.scale_num].scale[(int) ps->p.exact_lry][(int) ps->p.exact_y][
                                     (int) ps->p.exact_x +
                                     1] -
                             LoG.layers[ps->p.scale_num].scale[(int) ps->p.exact_lry][(int) ps->p.exact_y][
                                     (int) ps->p.exact_x - 1];
                        dy = LoG.layers[ps->p.scale_num].scale[(int) ps->p.exact_lry][(int) ps->p.exact_y +
                                                                                      1][(int) ps->p.exact_x] -
                             LoG.layers[ps->p.scale_num].scale[(int) ps->p.exact_lry][(int) ps->p.exact_y -
                                                                                      1][(int) ps->p.exact_x];
                        mod = sqrt(dx * dx + dy * dy);
                        ori = atan(dy / dx);
                        double weight, bin;
                        weight = exp(-1 * (p * p + q * q) / (2 * sigma * sigma)) / (2 * PI * sigma);
                        bin = round(KEYPOINT_DESCRIPTOR_HIST_BIN * (ori + PI) / (2.0 * PI));
                        bin = (bin < KEYPOINT_DESCRIPTOR_HIST_BIN) ? bin : 0;
                        pd->oriens[(int) (i * 8 + bin)].weight += weight;
                        pd->oriens[(int) (i * 8 + bin)].theta = bin;
                    }
                }
            }
        }
    }
    return 0;
}

SIFT::SIFT(int _width, int _height, int _channel) {
    width = _width;
    height = _height;
    channel = _channel;
    ekps = new Point;
    ekps->next = NULL;
    kps = new finalKeyPoint;
    kps->next = NULL;
    img = new uchar **[height];
    int i, j;
    for (i = 0; i != height; ++i) {
        img[i] = new uchar *[width];
        for (j = 0; j != width; ++j) {
            img[i][j] = new uchar[channel];
        }
    }
    img_gray = new uchar *[height];
    for (i = 0; i != height; ++i) {
        img_gray[i] = new uchar[width];
    }
}

SIFT::~SIFT() {
    delete[] kps;
    int i, j;
    for (i = 0; i != height; ++i) {
        for (j = 0; j != width; ++j) {
            delete[] img[i][j];
        }
        delete[] img[i];
    }
    for (i = 0; i != width; ++i) {
        delete[] img_gray[i];
    }
}

int SIFT::SetImg(uchar ***_img) {
    int i, j, k;
    for (i = 0; i != height; ++i) {
        for (j = 0; j != width; ++j) {
            for (k = 0; k != channel; ++k) {
                img[i][j][k] = _img[i][j][k];
            }
        }
    }
    return 0;
}

int SIFT::SIFT_createDoG() {
    rgb2gray(img, img_gray, width, height);
    buildLoGPyramid(img_gray, LoG, width, height, (log(min(width, height)) / log(2.0) - 2));
    buildDoGPyramid(LoG, DoG);
    return 0;
}

int SIFT::SIFT_searchKeyPoints() {
    Point *key = new Point;
    key->next = NULL;
    findDoGExtreme(DoG, key);
    Point *ekey = new Point;
    ekey->next = NULL;
    findExactKeypoint(LoG, key, ekey);
    Point *cekey = new Point;
    cekey->next = NULL;
    checkContrastKeypoint(LoG, ekey, cekey);
    adjustExtremePoints(LoG, cekey, ekps);
    return 0;
}

int SIFT::SIFT_createDescriptor() {
    finalKeyPoint *key = new finalKeyPoint;
    key->next = NULL;
    buildFirstOrientation(LoG, ekps, key);
    buildKeyPointDescriptor(LoG, key, kps);
    return 0;
}

int SIFT::run_sift() {
    SIFT_createDoG();
    SIFT_searchKeyPoints();
    SIFT_createDescriptor();
}

finalKeyPoint *SIFT::GetFinalKeypoint() {
    finalKeyPoint *ans = new finalKeyPoint;
    finalKeyPoint *pd = ans;
    pd->next = NULL;
    finalKeyPoint *ps = kps;
    while (ps->next) {
        ps = ps->next;
        pd->next = new finalKeyPoint;
        pd = pd->next;
        pd->next = NULL;
        pd->first_ori = new Orientation[ps->first_ori_num];
        pd->first_ori_num = ps->first_ori_num;
        for (int i = 0; i != pd->first_ori_num; ++i) {
            pd->first_ori[i] = ps->first_ori[i];
        }
        pd->p = ps->p;
        for (int i = 0; i != KEYPOINT_FEATURE_DESCRIPTOR_NUMBER; ++i) {
            pd->oriens[i] = ps->oriens[i];
        }
    }
    return ans;
}