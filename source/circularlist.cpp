#include "circularlist.h"

#include <iostream>
#include <cinder/app/AppBasic.h>

void add2DCircularList(DCircularList **list, char name, float u, float v)
{
    DCircularList *newNode = new DCircularList;
    newNode->name	= name;
    newNode->u		= u;
    newNode->v		= v;
    if(*list == 0) {
        *list = newNode;
        (*list)->next = (*list);
        (*list)->prev = (*list);
        return;
    } else {
        DCircularList *iter = *list;
        while(iter->next != (*list)) iter = iter->next;
        newNode->prev	= iter;
        newNode->next	= (*list);
        iter->next		= newNode;
        (*list)->prev	= newNode;
    }
}

DCircularList* rotateDCircularListFrwd(DCircularList *list, int moves)
{
    DCircularList *returnVal = list;
    for(int step=0; step < moves; ++step)
        returnVal = returnVal->next;
    return returnVal;
}

int findRotateStepsFrwd(DCircularList *list, DCircularList *node)
{
    DCircularList *iter = list;
    int stepCount=0;
    while( iter != list ) {
        if( iter == node )
            break;
        iter = iter->next;
        ++stepCount;
    }
    return stepCount;
}

DCircularList* rotateDCircularListBckwrd(DCircularList *list, int moves)
{
    DCircularList *returnVal = list;
    for(int step=0; step < moves; ++step)
        returnVal = returnVal->prev;
    return returnVal;
}

int findRotateStepsBckwrd(DCircularList *list, DCircularList *node)
{
    DCircularList *iter = list;
    int stepCount=0;
    while( iter != list ) {
        if( iter == node )
            break;
        iter = iter->prev;
        ++stepCount;
    }
    return stepCount;
}

void showDCircularList(DCircularList *list)
{
    DCircularList *iter = list;
    do {
        ci::app::console()<<"Item : "<<iter->name<<" "<<iter->u<<","<<iter->v;
        iter = iter->next;
    } while(iter != list);
    ci::app::console()<<std::endl;
}
