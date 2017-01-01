//
// Created by SXY on 2017-01-01.
//

#ifndef SIFT_MATCH_UTILS_H
#define SIFT_MATCH_UTILS_H

#define KEYPOINT_FEATURE_DESCRIPTOR_NUMBER 128
#define KEYPOINT_DESCRIPTOR_RADIUS 16
#define KEYPOINT_DESCRIPTOR_HIST_BIN 8
#define SIFT_SIGMA 1.6
#define SIFT_SIGMA_INITIAL 0.5
#define GAUSSIAN_MODAL_SIZE (int)(7*SIFT_SIGMA+1)
#define SIFT_KEYPOINT_BORDER GAUSSIAN_MODAL_SIZE
#define DEFAULT_SCALE_LAYER_NUMBER 6
#define SIFT_CONTRAST_THETA 0.04
#define SIFT_EDGE_R 10
#define SIFT_ORI_RADIUS 3*SIFT_SIGMA
#define SIFT_ORI_PEAK_RAD 0.8
#define SIFT_HIST_BIN 36
#define HIST_SMOOTH_PASSES 2

#define PI 3.14159265758979323846
#define min(x, y) (((x)<(y))?(x):(y))
#define max(a, b) (((a)>(b))?(a):(b))

typedef unsigned char uchar;

struct scaleSpace {
    uchar ***scale;
    int lry_num;
    int width, height;
};

struct Pyramid {
    scaleSpace *layers;
    int octave;
};

struct Point {
    int scale_x, scale_y;
    int img_x, img_y;
    double exact_x, exact_y, exact_lry;
    int scale_num, lry_num;
    uchar value;
    Point *next;
};

struct Orientation {
    double theta;
    double weight;
};

struct finalKeyPoint {
    Point p;
    Orientation *first_ori;
    int first_ori_num;
    Orientation oriens[KEYPOINT_FEATURE_DESCRIPTOR_NUMBER];
    finalKeyPoint *next;
};

typedef struct kt {
    finalKeyPoint node_data;
    kt *left, *right, *parent;
    int split;
} kdt;

typedef struct p_Kt {
    kdt *node;
    double dist;
    p_Kt *next;
} p_Kdt;

struct match_line {
    Point p1;
    Point p2;
    double dist;
};

#endif //SIFT_MATCH_UTILS_H
