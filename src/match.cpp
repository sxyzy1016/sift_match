//
// Created by SXY on 2017-01-01.
//

#include "match.h"

int match::setKdtree(kdtree _tree1, kdtree _tree2) {
    tree1 = _tree1;
    tree2 = _tree2;
    return 0;
}

int match::run_match() {
    match_line *ans = new match_line[min(tree1.GetKpNum(), tree2.GetKpNum())];
    mth_num = 0;
    for (int i = 0; i != tree1.GetKpNum(); ++i) {
        finalKeyPoint *p = tree1.GetKpfromIndex(i);
        finalKeyPoint *fn, *sn;
        double fd, sd;
        tree2.searchKdtree(p, fn, sn, fd, sd);
        if (fd / sd < 0.8) {
            ans[mth_num].p1 = p->p;
            ans[mth_num].p2 = fn->p;
            ans[mth_num].dist = fd;
            ++mth_num;
        }
    }
    mth_line = new match_line[mth_num + 1];
    for (int i = 0; i != mth_num + 1; ++i) {
        mth_line[i] = ans[i];
    }
    return 0;
}

int match::GetMatchLinesNum() {
    return mth_num;
}

match_line *match::GetMatchLines() {
    match_line *ans = new match_line[mth_num];
    for (int i = 0; i != mth_num + 1; ++i) {
        ans[i] = mth_line[i];
    }
    return ans;
}
