//
// Created by SXY on 2016-12-29.
//

#ifndef SIFT_MATCH_SIFT_H
#define SIFT_MATCH_SIFT_H

#include "utils.h"

class SIFT {
private:
    uchar ***img;
    uchar **img_gray;
    int width, height, channel;
    Pyramid LoG, DoG;
    Point *ekps;
    finalKeyPoint *kps;

public:
    /*构造函数*/
    SIFT(int _width, int _height, int _channel);

    ~SIFT();

    int SetImg(uchar ***_img);

    /*构造DOG尺度空间*/
    int SIFT_createDoG();

    //三通道转灰度
    int rgb2gray(uchar ***src, uchar **dst, double _width, double _height);

    //高斯滤波
    int gaussianBlur(uchar **src, uchar **dst, double _width, double _height, double sigma);

    //构造尺度空间
    int buildScaleSpace(uchar **src, scaleSpace &dst, double _width, double _height, double _lry_num);

    //构造LoG金字塔
    int buildLoGPyramid(uchar **src, Pyramid &dst, double _width, double _height, double _octave);

    //构造DoG差分金字塔
    int buildDoGPyramid(Pyramid src, Pyramid &dst);

    /*搜索关键点*/
    int SIFT_searchKeyPoints();

    //搜索尺度空间极值点
    int findScaleSpaceExtreme(scaleSpace src, Point *keypoint, int _scale_num);

    //搜索DoG局部极值点
    int findDoGExtreme(Pyramid src, Point *keypoint);

    //极值点精确定位
    int findExactKeypoint(Pyramid LoG, Point *src, Point *dst);

    //去除低对比度点
    int checkContrastKeypoint(Pyramid LoG, Point *src, Point *dst);

    //去除边缘响应
    int adjustExtremePoints(Pyramid LoG, Point *src, Point *dst);

    /*生成关键点描述子*/
    int SIFT_createDescriptor();

    //生成关键点主方向
    int buildFirstOrientation(Pyramid LoG, Point *src, finalKeyPoint *dst);

    //生成关键点描述子
    int buildKeyPointDescriptor(Pyramid LoG, finalKeyPoint *src, finalKeyPoint *dst);

    //运行SIFT算法
    int run_sift();

    finalKeyPoint *GetFinalKeypoint();
};

#endif //SIFT_MATCH_SIFT_H
