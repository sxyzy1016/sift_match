//
// Created by SXY on 2017-01-01.
//

#include <cstdio>
#include <cmath>
#include "kdtree.h"

using namespace std;

int kdtree::setFinalKeyPoints(finalKeyPoint *src) {
    finalKeyPoint *ps = src;
    finalKeyPoint *pd = kspl;
    kspn = 0;
    while (ps->next) {
        ps = ps->next;
        pd->next = new finalKeyPoint;
        pd = pd->next;
        pd->first_ori = new Orientation[ps->first_ori_num];
        pd->next = NULL;
        pd->first_ori_num = ps->first_ori_num;
        for (int i = 0; i != KEYPOINT_FEATURE_DESCRIPTOR_NUMBER; ++i) {
            pd->oriens[i] = ps->oriens[i];
        }
        pd->p = ps->p;
        for (int i = 0; i != pd->first_ori_num; ++i) {
            pd->first_ori[i] = ps->first_ori[i];
        }
    }
    return 0;
}

int kdtree::changeKeyPointsStruct() {
    ksp = new finalKeyPoint[kspn];
    finalKeyPoint *ps = kspl->next;
    for (int i = 0; i != kspn; ++i) {
        ksp[i].next = NULL;
        ksp[i].first_ori = new Orientation[ps->first_ori_num];
        ksp[i].first_ori_num = ps->first_ori_num;
        ksp[i].p = ps->p;
        for (int j = 0; j != ps->first_ori_num; ++j) {
            ksp[i].first_ori[j] = ps->first_ori[j];
        }
        for (int j = 0; j != KEYPOINT_FEATURE_DESCRIPTOR_NUMBER; ++j) {
            ksp[i].oriens[j] = ps->oriens[j];
        }
        ps = ps->next;
    }
    return 0;
}

int kdtree::QSort(finalKeyPoint *tt, int s, int t, int now) {
    int i = s, j = t;
    if (s < t) {
        finalKeyPoint tmp = tt[s];
        while (i != j) {
            while (j > i && tt[j].oriens[now].weight >= tmp.oriens[now].weight) --j;
            tt[i] = tt[j];
            while (i < j && tt[i].oriens[now].weight <= tmp.oriens[now].weight) ++i;
            tt[j] = tt[i];
        }
        tt[i] = tmp;
        QSort(tt, s, i - 1, now);
        QSort(tt, i + 1, t, now);
    }
    return 0;
}

double kdtree::dist(finalKeyPoint a, finalKeyPoint b) {
    double ax = a.p.img_x, ay = a.p.img_y;
    double bx = b.p.img_x, by = a.p.img_y;
    double d = sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
    return d;
}

int kdtree::delpdt(kdt *t) {
    int i = delpdt(t->left);
    int j = delpdt(t->right);
    delete t;
    return (int) (i && j);
}

int kdtree::p_createKdtree(finalKeyPoint *src, kdt *tree, int kp_start, int kp_end, kdt *parent) {
    int kp_num = kp_end - kp_start;
    if (kp_num < 0) {
        return 1;
    }
    int i, j;
    tree->parent = parent;
    tree->left = NULL;
    tree->right = NULL;
    double sagm[KEYPOINT_FEATURE_DESCRIPTOR_NUMBER];
    for (i = 0; i != KEYPOINT_FEATURE_DESCRIPTOR_NUMBER; ++i) {
        double sum_e = 0, sum_e2 = 0;
        for (j = kp_start; j != kp_end + 1; ++j) {
            sum_e += src[j].oriens[i].weight;
            sum_e2 += src[j].oriens[i].weight * src[j].oriens[i].weight;
        }
        sagm[i] = sum_e2 / (kp_num + 1) - (sum_e / (kp_num + 1)) * (sum_e / (kp_num + 1));
    }
    tree->split = 0;
    double max_sagm = sagm[0];
    for (i = 0; i != KEYPOINT_FEATURE_DESCRIPTOR_NUMBER; ++i) {
        if (sagm[i] > max_sagm) {
            tree->split = i;
            max_sagm = sagm[i];
        }
    }
    this->QSort(src, kp_start, kp_end, tree->split);
    int kp_mid = kp_start + (int) (kp_num / 2);
    tree->node_data = src[kp_mid];
    tree->left = new kdt;
    tree->right = new kdt;
    if (this->p_createKdtree(src, tree->left, kp_start, kp_mid - 1, tree)) {
        tree->left = NULL;
    }
    if (this->p_createKdtree(src, tree->right, kp_mid + 1, kp_end, tree)) {
        tree->right = NULL;
    }
    return 0;
}

int kdtree::p_searchKdtree(kdt *tree, finalKeyPoint *target, p_Kdt *first_nearest, p_Kdt *second_nearest) {
    if (!tree->left && !tree->right) {
        first_nearest->node = tree;
        first_nearest->dist = dist(*target, tree->node_data);
        second_nearest->node = tree;
        second_nearest->dist = dist(*target, tree->node_data);
        return 0;
    }
    p_Kdt *p_bot, *p_top;
    p_top = new p_Kdt;
    p_bot = p_top;
    p_bot->next = NULL;
    kdt *k = tree;
    while (k->left || k->right) {
        p_top->node = k;
        p_top->dist = dist(*target, k->node_data);
        p_Kdt *in = new p_Kdt;
        in->next = p_top;
        p_top = in;
        if (target->oriens[k->split].weight <= k->node_data.oriens[k->split].weight) {
            if (!k->left) {
                break;
            } else {
                k = k->left;
            }
        } else {
            if (!k->right) {
                break;
            } else {
                k = k->right;
            }
        }
    }
    p_top = p_top->next;
    p_Kdt *s_top = new p_Kdt;
    p_Kdt *s = s_top;
    s->next = NULL;
    double min_dist = p_top->dist;
    int kk = 0;
    while (p_top->next) {
        ++kk;
        if (!p_top->node->right && !p_top->node->left) {
            s->next = p_top;
            p_top = p_top->next;
            s = s->next;
            s->next = NULL;
            min_dist = min_dist > p_bot->dist ? p_top->dist : min_dist;
            continue;
        }
        if (p_top->node->node_data.oriens[p_top->node->split].weight + min_dist >= p_top->node->split ||
            p_top->node->node_data.oriens[p_top->node->split].weight - min_dist <= p_top->node->split) {
            kdt *next_tree = (target->oriens[p_top->node->split].weight <=
                              p_top->node->node_data.oriens[p_top->node->split].weight) ? p_top->node->right
                                                                                        : p_top->node->left;
            if (next_tree) {
                p_Kdt *fn = new p_Kdt;
                p_Kdt *sn = new p_Kdt;
                p_searchKdtree(next_tree, target, fn, sn);
                s->next = p_top;
                p_top = p_top->next;
                s = s->next;
                s->next = NULL;
                sn->next = p_top->next;
                p_top = sn;
                fn->next = p_top->next;
                p_top = fn;
            } else {
                s->next = p_top;
                p_top = p_top->next;
                s = s->next;
                s->next = NULL;
                continue;
            }
        } else {
            s->next = p_top;
            p_top = p_top->next;
            s = s->next;
            s->next = NULL;
        }
    }
    s = s_top->next;
    int first_index = 0, i = 0;
    first_nearest = s;
    s = s->next;
    while (s) {
        if (s->dist < first_nearest->dist) {
            first_nearest = s;
            first_index = i;
        }
        s = s->next;
        ++i;
    }
    s = s_top->next;
    i = 0;
    second_nearest = s;
    s = s->next;
    while (s) {
        if (s->dist < second_nearest->dist && first_index != i) {
            second_nearest = s;
        }
        s = s->next;
        ++i;
    }
    return 0;
}

kdt *kdtree::p_gettree(kdt *now, kdt *parent) {
    if (!now) {
        return now;
    }
    kdt *ans = new kdt;
    ans->split = now->split;
    ans->node_data = now->node_data;
    ans->parent = parent;
    ans->left = p_gettree(now->left, ans);
    ans->right = p_gettree(now->right, ans);
    return ans;
}

int kdtree::createKdtree() {
    return p_createKdtree(ksp, ptree, 0, kspn, NULL);
}

int kdtree::searchKdtree(finalKeyPoint *target, finalKeyPoint *&first_nearest, finalKeyPoint *&second_nearest,
                         double &first_dist, double &second_dist) {
    p_Kdt *fn = new p_Kdt;
    p_Kdt *sn = new p_Kdt;
    p_searchKdtree(ptree, target, fn, sn);
    first_nearest = new finalKeyPoint;
    *first_nearest = fn->node->node_data;
    second_nearest = new finalKeyPoint;
    *second_nearest = sn->node->node_data;
    first_dist = fn->dist;
    second_dist = sn->dist;
    return 0;
}

int kdtree::GetKpNum() {
    return kspn;
}

finalKeyPoint *kdtree::GetKpfromIndex(int index) {
    finalKeyPoint *ans = new finalKeyPoint;
    ans->first_ori = new Orientation[ksp[index].first_ori_num];
    ans->first_ori_num = ksp[index].first_ori_num;
    ans->p = ksp[index].p;
    ans->next = NULL;
    for (int i = 0; i != ans->first_ori_num; ++i) {
        ans->first_ori[i] = ksp[index].first_ori[i];
    }
    for (int i = 0; i != KEYPOINT_FEATURE_DESCRIPTOR_NUMBER; ++i) {
        ans->oriens[i] = ksp[index].oriens[i];
    }
    return ans;
}

kdt *kdtree::GetTree() {
    if (!ptree) {
        return NULL;
    }
    kdt *ans = new kdt;
    ans->node_data = ptree->node_data;
    ans->parent = NULL;
    ans->split = ptree->split;
    ans->left = p_gettree(ptree->left, ans);
    ans->right = p_gettree(ptree->right, ans);
    return ans;
}

kdtree::kdtree() {
    ksp = NULL;
    kspl = new finalKeyPoint;
    kspn = 0;
    ptree = new kdt;
}

kdtree::~kdtree() {
    delete[] ksp;
    finalKeyPoint *p1 = kspl, *p2 = kspl->next;
    while (p2) {
        delete p1;
        p1 = p2;
        p2 = p2->next;
    }
    delete p1;
    delpdt(ptree);
}