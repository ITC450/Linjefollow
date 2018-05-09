//
// Created by jeppe on 09-05-18.
//
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>       /* time */
#include "matx.h"
#include "neu.h"

void
load_bpenetTxT(mlp_net ** netvark, int *antinput, int *antneuroner, int *antoutput)
{
    if ((*netvark = (mlp_net *) malloc(sizeof(mlp_net))) == NULL)
        Error(1);
    loadMattxt("w1.txt", &((*netvark)->inw));
    loadMattxt("w2.txt", &((*netvark)->udw));
    loadMattxt("off1.txt", &((*netvark)->inoff));
    loadMattxt("off2.txt", &((*netvark)->udoff));
    *antinput = ((*netvark)->inw)->cols;
    *antneuroner = ((*netvark)->inw)->rows;
    *antoutput = ((*netvark)->udw)->rows;
    initmat(&((*netvark)->inwmel), *antneuroner, *antinput, 0.0);
    initmat(&((*netvark)->udwmel), *antoutput, *antneuroner, 0.0);
    initmat(&((*netvark)->fejl1), *antneuroner, 1, 0.0);
    initmat(&((*netvark)->fejl2), *antoutput, 1, 0.0);
    initmat(&((*netvark)->out0), 1, *antinput, 0.0);
    initmat(&((*netvark)->out1), 1, *antneuroner, 0.0);
    initmat(&((*netvark)->melres1), *antneuroner, 1, 0.0);
    initmat(&((*netvark)->melres2), *antneuroner, 1, 0.0);
    initmat(&((*netvark)->melres3), *antoutput, 1, 0.0);
    initmat(&((*netvark)->indwt), *antinput, *antneuroner, 0.0);
    initmat(&((*netvark)->udwt), *antneuroner, *antoutput, 0.0);
}

int main(){
    mlp_net *nn_net;
    matrix *uext;
    int neu = 10;                                              //Antal neuroner
    int input = 256;                                           //Antal input til netværket
    int output = 9;
    load_bpenetTxT(&nn_net, &input, &neu, &output);
    loadMattxt("uext.txt",&uext);
    save_bpenet("DataNet.nn", nn_net);
    saveMatD("uext",uext);
}
