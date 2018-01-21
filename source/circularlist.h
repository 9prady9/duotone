#pragma once

class DCircularList
{
    public:
        char name;
        float u;
        float v;
        DCircularList *next;
        DCircularList *prev;

        ~DCircularList() {
            delete next;
            delete prev;
        }
};

void add2DCircularList(DCircularList **list, char name, float u, float v);

DCircularList* rotateDCircularListFrwd(DCircularList *list, int moves);

int findRotateStepsFrwd(DCircularList *list, DCircularList *node);

DCircularList* rotateDCircularListBckwrd(DCircularList *list, int moves);

int findRotateStepsBckwrd(DCircularList *list, DCircularList *node);

void showDCircularList(DCircularList *list);
