//
// Created by SXY on 2017-01-01.
//

#ifndef SIFT_MATCH_KDTREE_H
#define SIFT_MATCH_KDTREE_H

#include "utils.h"

class kdtree {
private:
    finalKeyPoint *ksp;
    finalKeyPoint *kspl;
    int kspn;
    kdt *ptree;

    int p_createKdtree(finalKeyPoint *src, kdt *tree, int kp_start, int kp_end, kdt *parent);

    int p_searchKdtree(kdt *tree, finalKeyPoint *target, p_Kdt *first_nearest, p_Kdt *second_nearest);

    kdt *p_gettree(kdt *now, kdt *parent);

    int QSort(finalKeyPoint *tt, int s, int t, int now);

    double dist(finalKeyPoint a, finalKeyPoint b);

    int delpdt(kdt *t);

public:
    //构造函数
    kdtree();

    ~kdtree();

    //设置数据
    int setFinalKeyPoints(finalKeyPoint *src);

    //调整关键点数据结构
    int changeKeyPointsStruct();

    //构造KD树
    int createKdtree();

    //从KD树中搜索最近和次近的点
    int searchKdtree(finalKeyPoint *target, finalKeyPoint *&first_nearest, finalKeyPoint *&second_nearest,
                     double &first_dist, double &second_dist);

    int GetKpNum();

    finalKeyPoint *GetKpfromIndex(int index);

    kdt *GetTree();
};

#endif //SIFT_MATCH_KDTREE_H
