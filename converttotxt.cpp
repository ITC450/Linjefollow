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
save_bpenetTxT(mlp_net *netvark) {
    saveMattxt("w1.txt", netvark->inw);
    saveMattxt("w2.txt", netvark->udw);
    saveMattxt("off1.txt", netvark->inoff);
    saveMattxt("off2.txt", netvark->udoff);
}

int main() {
    mlp_net *nn_net;
    matrix *uext;
    int neu = 10;                                              //Antal neuroner
    int input = 256;                                           //Antal input til netværket
    int output = 9;
    load_bpenet("DataNet.nn", &nn_net, &input, &neu, &output);
    loadMatD("uext", &uext);
    save_bpenetTxT(nn_net);
    saveMattxt("uext.txt",uext);
    return 0;
}
