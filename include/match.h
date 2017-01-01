//
// Created by SXY on 2017-01-01.
//

#ifndef SIFT_MATCH_MATCH_H
#define SIFT_MATCH_MATCH_H

#include "utils.h"
#include "SIFT.h"
#include "kdtree.h"

class match {
private:
    kdtree tree1, tree2;
    match_line *mth_line;
    int mth_num;

public:

    int setKdtree(kdtree _tree1, kdtree _tree2);

    int run_match();

    int GetMatchLinesNum();

    match_line *GetMatchLines();
};

#endif //SIFT_MATCH_MATCH_H
