#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "matx.h"
#include "neu.h"

void
bpe_forward(matrix * netind, mlp_net * netvark, matrix ** netud);

/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/

void
ligeinit_bpe(mlp_net ** netvark, int antinput, int antneuroner, int antoutput, float min, float max)
{
    if ((*netvark = (mlp_net *) malloc(sizeof(mlp_net))) == NULL)
        Error(1);
    randinitmat(&((*netvark)->inw), antneuroner, antinput, min, max);
    randinitmat(&((*netvark)->udw), antoutput, antneuroner, min, max);
    initmat(&((*netvark)->inwmel), antneuroner, antinput, 0.0);
    initmat(&((*netvark)->udwmel), antoutput, antneuroner, 0.0);
    randinitmat(&((*netvark)->inoff), antneuroner, 1, min, max);
    randinitmat(&((*netvark)->udoff), antoutput, 1, min, max);
    initmat(&((*netvark)->fejl1), antneuroner, 1, 0.0);
    initmat(&((*netvark)->fejl2), antoutput, 1, 0.0);
    initmat(&((*netvark)->out0), 1, antinput, 0.0);
    initmat(&((*netvark)->out1), 1, antneuroner, 0.0);
    initmat(&((*netvark)->melres1), antneuroner, 1, 0.0);
    initmat(&((*netvark)->melres2), antneuroner, 1, 0.0);
    initmat(&((*netvark)->melres3), antoutput, 1, 0.0);
    initmat(&((*netvark)->indwt), antinput, antneuroner, 0.0);
    initmat(&((*netvark)->udwt), antneuroner, antoutput, 0.0);
}



/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/

void
mlp_gaussinit(mlp_net ** netvark, int antinput, int antneuroner, int antoutput, float my, float sigma)
{
    if ((*netvark = (mlp_net *) malloc(sizeof(mlp_net))) == NULL)
        Error(1);
    gaussinitmat(&((*netvark)->inw), antneuroner, antinput, my, sigma);
    gaussinitmat(&((*netvark)->udw), antoutput, antneuroner, my, sigma);
    initmat(&((*netvark)->inwmel), antneuroner, antinput, 0.0);
    initmat(&((*netvark)->udwmel), antoutput, antneuroner, 0.0);
    gaussinitmat(&((*netvark)->inoff), antneuroner, 1, my, sigma);
    gaussinitmat(&((*netvark)->udoff), antoutput, 1, my, sigma);
    initmat(&((*netvark)->fejl1), antneuroner, 1, 0.0);
    initmat(&((*netvark)->fejl2), antoutput, 1, 0.0);
    initmat(&((*netvark)->out0), 1, antinput, 0.0);
    initmat(&((*netvark)->out1), 1, antneuroner, 0.0);
    initmat(&((*netvark)->melres1), antneuroner, 1, 0.0);
    initmat(&((*netvark)->melres2), antneuroner, 1, 0.0);
    initmat(&((*netvark)->melres3), antoutput, 1, 0.0);
    initmat(&((*netvark)->indwt), antinput, antneuroner, 0.0);
    initmat(&((*netvark)->udwt), antneuroner, antoutput, 0.0);

}



/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/

void
kill_bpenet(mlp_net ** netvark)
{
    killmat(&((*netvark)->inw));
    killmat(&((*netvark)->udw));
    killmat(&((*netvark)->inwmel));
    killmat(&((*netvark)->udwmel));
    killmat(&((*netvark)->inoff));
    killmat(&((*netvark)->udoff));
    killmat(&((*netvark)->fejl1));
    killmat(&((*netvark)->fejl2));
    killmat(&((*netvark)->out0));
    killmat(&((*netvark)->out1));
    killmat(&((*netvark)->melres1));
    killmat(&((*netvark)->melres2));
    killmat(&((*netvark)->melres3));
    killmat(&((*netvark)->indwt));
    killmat(&((*netvark)->udwt));
    free(*netvark);

}



/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/

void
load_bpenet(char const *filnavn, mlp_net ** netvark, int *antinput, int *antneuroner, int *antoutput)
{
    FILE           *infil;
    if ((infil = fopen(filnavn, "r")) == NULL)
        Error(2);
    if ((*netvark = (mlp_net *) malloc(sizeof(mlp_net))) == NULL)
        Error(1);
    getmat(infil, "w1", &((*netvark)->inw));
    getmat(infil, "w2", &((*netvark)->udw));
    getmat(infil, "off1", &((*netvark)->inoff));
    getmat(infil, "off2", &((*netvark)->udoff));
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
    fclose(infil);
}



/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/

void
save_bpenet(char const *filnavn, mlp_net * netvark)
{
    FILE           *infil;
    if ((infil = fopen(filnavn, "w")) == NULL)
        Error(2);
    savemat(infil, "w1", netvark->inw);
    savemat(infil, "w2", netvark->udw);
    savemat(infil, "off1", netvark->inoff);
    savemat(infil, "off2", netvark->udoff);

    fclose(infil);
}


/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/

void
write_bpenet(mlp_net        *netvark)
{
    writemat("w1", netvark->inw);
    writemat("w2", netvark->udw);
    writemat("b1", netvark->inoff);
    writemat("b2", netvark->udoff);

}



/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/

void
bpe_forward(matrix * netind, mlp_net * netvark, matrix ** netud)
{
    int             i;
    mattrans(netind, &(netvark->out0));
    matmul(netvark->inw, netind, &(netvark->melres1));
    matsub(netvark->melres1, netvark->inoff, &(netvark->melres2));

    for (i = 0; i < (netvark->inw)->rows; i++)
    {
        elm(netvark->melres2, i, 0) = SIGMOID(elm(netvark->melres2, i, 0));
    }
    mattrans(netvark->melres2, &(netvark->out1));
    matmul(netvark->udw, netvark->melres2, &(netvark->melres3));
    matsub(netvark->melres3, netvark->udoff, netud);
}



/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/

void
bpe_backward(matrix * err, mlp_net * netvark, matrix ** err_on_ind)
{
    int             i;
    matassign(err, &(netvark->fejl2));
    mattrans(netvark->udw, &(netvark->udwt));
    matmul(netvark->udwt, err, &(netvark->melres1));

    for (i = 0; i < (netvark->inw)->rows; i++)
    {
        elm(netvark->fejl1, i, 0) = SIGMOIDdif(elm(netvark->melres2, i, 0)) *
                                    elm(netvark->melres1, i, 0);
    }

    mattrans(netvark->inw, &(netvark->indwt));
    matmul(netvark->indwt, netvark->fejl1, err_on_ind);
}



/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/

void
bpe_update(mlp_net * netvark, float alfa)
{
    int             i;
    matmulreal(netvark->fejl2, alfa, &(netvark->melres3));
    matmul(netvark->melres3, netvark->out1, &(netvark->udwmel));
    matsub(netvark->udw, netvark->udwmel, &(netvark->udw));
    matadd(netvark->udoff, netvark->melres3, &(netvark->udoff));
    matmulreal(netvark->fejl1, alfa, &(netvark->melres1));
    matmul(netvark->melres1, netvark->out0, &(netvark->inwmel));
    matsub(netvark->inw, netvark->inwmel, &(netvark->inw));
    matadd(netvark->inoff, netvark->melres1, &(netvark->inoff));
}


void
mlp_train(mlp_net * nn_net, matrix * xl, matrix * yl, int ant_itt, float alfa)
{
    matrix         *nninput, *nnoutput, *fejl, *xfejl, *fejlItt;
    float           sumfejl;
    int             i, j, k;
    initmat(&nninput, xl->cols, 1, 0.0);
    initmat(&nnoutput, yl->cols, 1, 0.0);
    initmat(&fejl, yl->cols, 1, 0.0);
    initmat(&xfejl, xl->cols, 1, 0.0);
    initmat(&fejlItt, (ant_itt)/10,yl->cols, 0.0);

    for (i = 1; i < ant_itt+1; i++)
    {
        sumfejl = 0.0;
        for (j = 0; j < xl->rows; j++)
        {
            for (k = 0; k < xl->cols; k++)
                elm(nninput, k, 0) = elm(xl, j, k);
            bpe_forward(nninput, nn_net, &nnoutput);


            for (k = 0; k < yl->cols; k++)
            {
                elm(fejl, k, 0) = elm(nnoutput, k, 0) - elm(yl, j, k);
                sumfejl += elm(fejl, k, 0) * elm(fejl, k, 0);
            }

            /* writemat("error",fejl); */
            bpe_backward(fejl, nn_net, &xfejl);
            bpe_update(nn_net, alfa);

        }

        if (i ==ant_itt)
        {
            sumfejl =
                sumfejl / (float) (xl->rows);
            printf("Error:  %e \n",
                   sumfejl);

        }
    }

    killmat(&nninput);
    killmat(&nnoutput);
    killmat(&fejl);
    killmat(&xfejl);
}

/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/

void CalcPerf(mlp_net *nn_net, matrix **perf, matrix *inLearning, matrix *outLearning, int Epock)
{
    matrix *nninput, *nnoutput;
    initmat(&nninput,   inLearning->cols,  1, 0.0);
    initmat(&nnoutput,  outLearning->cols, 1, 0.0);

    for (int j=0; j < inLearning->cols; j++)
    {
        for (int k = 0; k < inLearning->cols; k++)
            elm(nninput, k, 0) = elm(inLearning, j, k);
        bpe_forward(nninput, nn_net, &nnoutput);
        for (int l = 0; l < (*perf)->cols; l++)
            elm(*perf,Epock,l)+= (elm(nnoutput,l,0)- elm(outLearning,j,l))*(elm(nnoutput,l,0)- elm(outLearning,j,l));
    }
    for (int l = 0; l < (*perf)->cols; l++) elm(*perf,Epock,l)/= (inLearning->cols*2);
    killmat(&nninput);
    killmat(&nnoutput);
}

/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/


void
save_prazen(char const *filnavn, matrix * xl, matrix * yl, float h)
{
    FILE           *infil;
    if ((infil = fopen(filnavn, "w")) == NULL)
        Error(2);
    savemat(infil, "xl", xl);
    savemat(infil, "yl", yl);
    fwrite(&h, sizeof(float), 1, infil);
    fclose(infil);
}

void
load_prazen(char const *filnavn, matrix ** xl, matrix ** yl, float *h)
{
    FILE           *infil;
    if ((infil = fopen(filnavn, "r")) == NULL)
        Error(2);
    getmat(infil, "xl", xl);
    getmat(infil, "yl", yl);
    if (!fread(h, sizeof(float), 1, infil)) Error(0);
    fclose(infil);
}

void
parzen(matrix * xl, matrix * yl, float h, matrix * x, matrix ** netud)
{
    double            tmp, den= 0.0;
    for (int n = 0; n < yl->cols; n++) elm(*netud, n, 0) = 0.0;
    for (int k = 0; k < xl->rows; k++)
    {
        tmp = 0.0;
        for (int l = 0; l < xl->cols; l++)
            tmp += (((elm(xl, k, l) - elm(x, l, 0)) * (elm(xl, k, l) - elm(x, l, 0))) / (2 * h * h));
        tmp = exp(-tmp);
        den += tmp;
        for (int n = 0; n < yl->cols; n++)
            elm(*netud, n, 0) += elm(yl, k, n) * tmp;
    }
    for (int n = 0; n < yl->cols; n++)
        elm(*netud, n, 0) = elm(*netud, n, 0) / den;
}



/*********************************************************************/
/* matrix         *un, *uext, *w, *y, *lampda;                  */
/* */
/* initmat(&un, ulaer->rows, ulaer->cols, 0.0);                 */
/* */
/* initmat(&uext, ulaer->cols, 2, 0.0);                         */
/* */
/* matnormer(ulaer, &un, &uext, 1);                             */
/* */
/* rce_train(un, ylaer, 10.0, &w, &lampda, &y);                 */
/* */
/*********************************************************************/

void
rce_train(matrix * xl, matrix * yl, float rinit, matrix ** w, matrix ** lampda, matrix ** y)
{
    int             lg = -1, lny, k, l, m, n, antN = 0;
    char            fundet = 0;
    float           afstand;

    matrix         *wtmp, *lampdatmp, *ytmp;

    initmat(&wtmp, xl->rows, xl->cols, 0.0);
    initmat(&lampdatmp, yl->rows, 1, 0.0);
    initmat(&ytmp, yl->rows, yl->cols, 0.0);
    lny = xl->rows;

    while (lg != lny)
    {

        for (k = 0; k < xl->rows; k++)
        {
            fundet = 0;
            if (antN > 0)
            {
                for (l = 0; l < antN; l++)
                {
                    afstand = 0.0;
                    for (m = 0; m < xl->cols; m++)
                    {
                        afstand += ((elm(wtmp, l, m) - elm(xl, k, m)) * (elm(wtmp, l, m) - elm(xl, k, m)));
                    }
                    afstand = sqrt(afstand);
                    for (n = 0; n < yl->cols; n++)
                    {
                        if ((afstand < elm(lampdatmp, l, 0)) && (elm(yl, k, n) != elm(ytmp, l, n)))
                            elm(lampdatmp, l, 0) = afstand - 1.0e-3;
                    }

                    if (afstand < elm(lampdatmp, l, 0))
                        fundet = 1;
                    for (n = 0; n < yl->cols; n++)
                    {
                        if ((afstand < elm(lampdatmp, l, 0)) && (elm(yl, k, n) != elm(ytmp, l, n)))
                            fundet = 0;
                    }


                }
            }
            if (fundet == 0)
            {
                for (n = 0; n < xl->cols; n++)
                    elm(wtmp, antN, n) = elm(xl, k, n);
                for (n = 0; n < yl->cols; n++)
                    elm(ytmp, antN, n) = elm(yl, k, n);
                elm(lampdatmp, antN, 0) = rinit;
                antN++;
            }
        }
        lg = lny;
        lny = antN;
        printf("No. of neurons: %d\n", antN);
    }

    initmat(w, antN, xl->cols, 0.0);
    initmat(lampda, antN, 1, 0.0);
    initmat(y, antN, yl->cols, 0.0);

    for (k = 0; k < xl->cols; k++)
        for (l = 0; l < antN; l++)
            elm(*w, l, k) = elm(wtmp, l, k);
    for (k = 0; k < yl->cols; k++)
        for (l = 0; l < antN; l++)
            elm(*y, l, k) = elm(ytmp, l, k);
    for (l = 0; l < antN; l++)
        elm(*lampda, l, 0) = elm(lampdatmp, l, 0);

    killmat(&wtmp);
    killmat(&ytmp);
    killmat(&lampdatmp);
}


/*********************************************************************/
/* rce_rum(w, lampda, y, nninput, &nnoutput, &conf, &antC);              */
/* */
/* */
/* for (k = 1; k <= utest->cols; k++)                           */
/* elm(nninput, k, 1) = elm(untst, j, k);               */
/* rce_run(w, lampda, y, nninput, &nnoutput, &conf, &antC);     */
/*********************************************************************/

void
rce_run(matrix * w, matrix * lampda, matrix * y, matrix * x, matrix ** nnoutput, int *conf, int *antC)
{
    int             i, j, k, l, m, n, antN;
    float           afstand;
    matrix         *ytmp, *ext;

    antN = w->rows;
    k = 0;

    initmat(&ytmp, y->rows, y->cols, 0.0);

    if (antN > 0)
    {
        for (l = 0; l < antN; l++)
        {
            afstand = 0.0;
            for (m = 0; m < w->cols; m++)
            {
                afstand += ((elm(w, l, m) - elm(x, m, 0)) * (elm(w, l, m) - elm(x, m, 0)));
            }
            afstand = sqrt(afstand);

            /* printf("afstand: %f\n",afstand); */

            if (afstand < elm(lampda, l, 0))
            {
                for (n = 0; n < y->cols; n++)
                    elm(ytmp, k, n) = elm(y, l, n);
                k++;

            }
        }

    }
    if (k == 0)
    {
        for (l = 0; l < y->cols; l++)
            elm(*nnoutput, l, 0) = 0;
        *conf = 1;
    }
    if (k == 1)
    {
        for (l = 0; l < y->cols; l++)
            elm(*nnoutput, l, 0) = elm(ytmp, k - 1, l);
        *conf = 0;
    }
    if (k > 1)
    {
        initmat(&ext, ytmp->cols, 2, 0.0);

        for (j = 0; j < ytmp->cols; j++)
        {
            elm(ext, j, 1) = elm(ytmp, 0, j);
        }

        for (i = 0; i < k; i++)
            for (j = 0; j < ytmp->cols; j++)
            {
                if ((elm(ext, j, 1) < elm(ytmp, i, j)) || (elm(ext, j, 1) > elm(ytmp, i, j)))
                    *conf = 1;
                elm(ext, j, 0) += elm(ytmp, i, j);
            }

        for (l = 0; l < y->cols; l++)
            elm(*nnoutput, l, 0) = elm(ext, l, 0) / (float) k;
        killmat(&ext);
        *conf = 1;

    }
    *antC = k;
    killmat(&ytmp);
}


void
Sphere_init(sph_net ** nn_net, float rinit)
{

    if ((*nn_net = (sph_net *) malloc(sizeof(sph_net))) == NULL)
        Error(1);
    (*nn_net)->rinit = rinit;
}

void
Sphere_train(sph_net * nn_net, matrix * xl, matrix * yl)
{
    rce_train(xl, yl, nn_net->rinit, &(nn_net->w), &(nn_net->lampda), &(nn_net->y));
}

void
Sphere_run(sph_net * nn_net, matrix * x, matrix ** y, int *conf, int *antC)
{
    rce_run(nn_net->w, nn_net->lampda, nn_net->y, x, y, conf, antC);
}


void
load_sphnet(char const *filnavn, sph_net ** netvark)
{
    FILE           *infil;
    if ((infil = fopen(filnavn, "r")) == NULL)
        Error(2);
    if ((*netvark = (sph_net *) malloc(sizeof(sph_net))) == NULL)
        Error(1);
    getmat(infil, "w", &((*netvark)->w));
    getmat(infil, "lampda", &((*netvark)->lampda));
    getmat(infil, "y", &((*netvark)->y));
    (*netvark)->rinit = 10.0;
    fclose(infil);
}



/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/

void
save_sphnet(char const *filnavn, sph_net * netvark)
{
    FILE           *infil;
    if ((infil = fopen(filnavn, "w")) == NULL)
        Error(2);
    savemat(infil, "w", netvark->w);
    savemat(infil, "lampda", netvark->lampda);
    savemat(infil, "y", netvark->y);

    fclose(infil);
}

void
kill_sphnet(sph_net ** netvark)
{
    killmat(&((*netvark)->w));
    killmat(&((*netvark)->lampda));
    killmat(&((*netvark)->y));
    free(*netvark);

}


void
write_sphnet(sph_net        *netvark)
{
    writemat("w", netvark->w);
    writemat("lampda", netvark->lampda);
    writemat("y", netvark->y);
}



void
load_lpffnet(char const *filnavn, lpff_net ** netvark)
{
    FILE           *infil;
    int             k = 1, i, j;
    if ((infil = fopen(filnavn, "r")) == NULL)
        Error(2);
    if ((*netvark = (lpff_net *) malloc(sizeof(lpff_net))) == NULL)
        Error(1);
    getmat(infil, "w", &((*netvark)->w));
    getmat(infil, "antff", &((*netvark)->antff));
    getmat(infil, "ext", &((*netvark)->ext));
    getmat(infil, "b", &((*netvark)->b));
    getmat(infil, "del", &((*netvark)->del));

    for (i = 0; i < (*netvark)->antff->rows; i++)
        k = k * elm((*netvark)->antff, i, 0);

    initmat(&((*netvark)->out), k, 1, 0.0);

    i = 0;
    for (j = 0; j < (*netvark)->antff->rows; j++)
        if (i < (int) elm((*netvark)->antff, j, 0))
            i = (int) elm((*netvark)->antff, j, 0);

    initmat(&((*netvark)->ffout), i, (*netvark)->antff->rows, 0.0);

    fclose(infil);
}



/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/

void
save_lpffnet(char const *filnavn, lpff_net * netvark)
{
    FILE           *infil;
    if ((infil = fopen(filnavn, "w")) == NULL)
        Error(2);
    savemat(infil, "w", netvark->w);
    savemat(infil, "antff", netvark->antff);
    savemat(infil, "ext", netvark->ext);
    savemat(infil, "b", netvark->b);
    savemat(infil, "del", netvark->del);

    fclose(infil);
}

void
kill_lpffnet(lpff_net ** netvark)
{
    killmat(&((*netvark)->w));
    killmat(&((*netvark)->out));
    killmat(&((*netvark)->ffout));
    killmat(&((*netvark)->antff));
    killmat(&((*netvark)->ext));
    killmat(&((*netvark)->b));
    killmat(&((*netvark)->del));
    free(*netvark);

}

/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/
void
lpff_init(lpff_net ** nn_net, matrix * antff, matrix * xl, int antoutput, float kl)
{
    int             i, k = 1, j;


    if (antff->rows != xl->cols)
    {
        printf(" Spec. antallet af forfiltreringer svarre ikke til antallet af input\n");
        exit(1);
    }
    if ((*nn_net = (lpff_net *) malloc(sizeof(lpff_net))) == NULL)
        Error(1);

    for (i = 0; i < antff->rows; i++)
        k = k * elm(antff, i, 0);


    randinitmat(&((*nn_net)->w), antoutput, k, -0.1, 0.1);
    initmat(&((*nn_net)->out), k, 1, 0.0);
    initmat(&((*nn_net)->antff), antff->rows, 1, 0.0);
    matassign(antff, &((*nn_net)->antff));
    initmat(&((*nn_net)->b), antff->rows, 1, 0.0);
    initmat(&((*nn_net)->del), antff->rows, 1, 0.0);
    initmat(&((*nn_net)->ext), antff->rows, 2, 0.0);

    i = 0;
    for (j = 0; j < xl->cols; j++)
        if (i < (int) elm(antff, j, 0))
            i = (int) elm(antff, j, 0);

    initmat(&((*nn_net)->ffout), i, antff->rows, 0.0);
    for (j = 0; j < xl->cols; j++)
    {
        elm((*nn_net)->ext, j, 0) = elm(xl, 0, j);
        elm((*nn_net)->ext, j, 1) = elm(xl, 0, j);
    }

    for (j = 0; j < xl->cols; j++)
        for (i = 0; i < xl->rows; i++)
        {
            if (elm((*nn_net)->ext, j, 0) < elm(xl, i, j))
                elm((*nn_net)->ext, j, 0) = elm(xl, i, j);
            if (elm((*nn_net)->ext, j, 1) > elm(xl, i, j))
                elm((*nn_net)->ext, j, 1) = elm(xl, i, j);
        }

    for (j = 0; j < xl->cols; j++)
    {
        elm((*nn_net)->del, j, 0) = (elm((*nn_net)->ext, j, 0) - elm((*nn_net)->ext, j, 1)) / (elm(antff, j, 0) - 1);
        elm((*nn_net)->b, j, 0) = -(8.0 * log(kl)) / (elm((*nn_net)->del, j, 0) * elm((*nn_net)->del, j, 0));

    }
}


/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/
void
lpff_run(lpff_net * nn_net, matrix * x, matrix ** y)
{
    int             i, j;
    long            k, antcar = 1;

    int            *ffcnt;

    k = 1;
    ffcnt = (int *) malloc(sizeof(int) * x->rows + 1);

    for (i = 0; i < x->rows; i++)
    {
        antcar *= (long) elm(nn_net->antff, i, 0);
        ffcnt[i] = 0;
    }


    for (i = 0; i < x->rows; i++)
    {
        for (j = 0; j < (int) elm(nn_net->antff, i, 0); j++)
        {

            elm(nn_net->ffout, j, i) =
                exp(-0.5 * elm(nn_net->b, i, 0) * (elm(x, i, 0) - (elm(nn_net->ext, i, 1) + (float) (j) * elm(nn_net->del, i, 0))) *
                    (elm(x, i, 0) - (elm(nn_net->ext, i, 1) + (float) (j) * elm(nn_net->del, i, 0))));
        }
    }


    for (k = 0; k < antcar; k++)
    {
        elm(nn_net->out, k, 0) = 1.0;
        for (i = 0; i < x->rows; i++)
        {
            elm(nn_net->out, k, 0) *= elm(nn_net->ffout, ffcnt[i], i);
        }
        ffcnt[0]++;
        for (i = 0; i < x->rows; i++)
        {
            if (((ffcnt[i] % (((int) elm(nn_net->antff, i, 0)))) == 0) && (ffcnt[i]))
            {
                ffcnt[i] = 0;
                ffcnt[i + 1]++;

            }
        }
        /*	for (i = 0; i < x->rows; i++) {
        		printf("ffcnt : %d ", ffcnt[i]);
        	}
        	printf("\n");
        */


    }

    /*
     * whomat("w",nn_net->w); whomat("out",nn_net->out); whomat("y",*y);
     */

    matmul(nn_net->w, nn_net->out, y);
    free(ffcnt);
}


/*********************************************************************/
/* */
/* */
/* */
/* */
/* */
/*********************************************************************/
void
lpff_train(lpff_net * nn_net, matrix * xl, matrix * yl, char lernalgo, int antit, float alpha)
{
    matrix         *y, *ylv, *x, *tmpin, *err, *melres3, *wmel, *wmelt, *outt,
                   *G, *R, *Rtmp, *out2;
    int             i, l, k, m;
    long            antcar = 1;
    float           fejl, fejlold, rho;

    if (nn_net->w->rows != yl->cols)
    {
        printf(" Forkert ant outputs i lpff_init\n");
        exit(1);
    }
    if (nn_net->antff->rows != xl->cols)
    {
        printf(" Forkert ant input i forhold til det oenskede fra lpff_init\n");
        exit(1);
    }
    initmat(&x, xl->cols, 1, 0.0);
    initmat(&y, yl->cols, 1, 0.0);
    initmat(&ylv, yl->cols, 1, 0.0);
    initmat(&err, yl->cols, 1, 0.0);
    initmat(&melres3, yl->cols, 1, 0.0);
    initmat(&wmel, nn_net->w->rows, nn_net->w->cols, 0.0);

    initmat(&out2, nn_net->out->rows, nn_net->out->cols, 0.0);

    initmat(&outt, nn_net->out->cols, nn_net->out->rows, 0.0);
    if (lernalgo == LW_LEARN)
    {
        initmat(&wmelt, nn_net->w->cols, nn_net->w->rows, 0.0);
        initmat(&G, nn_net->out->rows, nn_net->out->cols, 0.0);
        initmat(&R, nn_net->out->rows, nn_net->out->rows, 0.0);
        initmat(&Rtmp, nn_net->out->rows, nn_net->out->rows, 0.0);
    }
    for (i = 0; i < xl->cols; i++)
    {
        antcar *= (long) elm(nn_net->antff, i, 0);
    }

    /*
     * initmat(&tmpin, antcar, xl->rows, 0.0);
     *
     * for (k = 1; k <= xl->rows; k++) {
     *
     * for (m = 1; m <= xl->cols; m++) { elm(x, m, 1) = elm(xl, k, m); }
     * lpff_run(nn_net, x, &y); for (i = 1; i <= antcar; i++) elm(tmpin,
     * i, k) = elm(nn_net->out, i, 1); }
     */


    rho = 100.0;
    fejl = 0.0;
    for (k = 0; k < xl->rows; k++)
    {
        for (m = 0; m < xl->cols; m++)
        {
            elm(x, m, 0) = elm(xl, k, m);
        }
        lpff_run(nn_net, x, &y);
        for (m = 0; m < yl->cols; m++)
        {
            elm(ylv, m, 0) = elm(yl, k, m);
        }
        matsub(y, ylv, &err);
        for (m = 0; m < yl->cols; m++)
            fejl += elm(err, m, 0) * elm(err, m, 0);
    }
    fejlold = fejl;
    for (l = 0; l < antit; l++)
    {
        if (lernalgo == BPE_LEARN)
        {
            fejlold = fejl;
            fejl = 0.0;
            for (k = 0; k < xl->rows; k++)
            {

                for (m = 0; m < xl->cols; m++)
                {
                    elm(x, m, 0) = elm(xl, k, m);
                }
                lpff_run(nn_net, x, &y);
                for (m = 0; m < yl->cols; m++)
                {
                    elm(ylv, m, 0) = elm(yl, k, m);
                }
                matsub(y, ylv, &err);
                matmulreal(err, alpha, &melres3);
                mattrans(nn_net->out, &outt);
                matmul(melres3, outt, &wmel);
                matsub(nn_net->w, wmel, &nn_net->w);

                for (m = 0; m < yl->cols; m++)
                    fejl += elm(err, m, 0) * elm(err, m, 0);
            }
            /*
             * if (fejl < fejlold) alpha *= 1.2; else if (alpha >
             * 1.0e-3) alpha *= 0.1;
             */
            if ((l % 100) == 0)
                printf("pt nr: %d antitt: %d  fejl: %f alpha: %f\n", k, l, fejl, alpha);
        }
        else
        {
            matzero(&G);
            matzero(&R);
            for (k = 0; k < xl->rows; k++)
            {

                for (m = 0; m < xl->cols; m++)
                {
                    elm(x, m, 0) = elm(xl, k, m);
                }
                lpff_run(nn_net, x, &y);
                for (m = 0; m < yl->cols; m++)
                {
                    elm(ylv, m, 0) = elm(yl, k, m);
                }
                matsub(y, ylv, &err);
                matmul(nn_net->out, err, &out2);
                matadd(out2, G, &G);
                mattrans(nn_net->out, &outt);
                matmul(nn_net->out, outt, &Rtmp);
                matadd(R, Rtmp, &R);
            }

            /* beregn P */
            fejl = 0.0;
            for (k = 0; k < xl->rows; k++)
            {
                for (m = 0; m < xl->cols; m++)
                {
                    elm(x, m, 0) = elm(xl, k, m);
                }
                lpff_run(nn_net, x, &y);
                for (m = 0; m < yl->cols; m++)
                {
                    elm(ylv, m, 0) = elm(yl, k, m);
                }
                matsub(y, ylv, &err);
                for (m = 0; m < yl->cols; m++)
                    fejl += elm(err, m, 0) * elm(err, m, 0);
            }
            printf("pt nr: %d antitt: %d  fejl: %f\n", k, l, fejl);
            fejlold = fejl + 1.0;
            rho *= 0.25;
            while (fejl < fejlold)
            {
                for (m = 0; m < R->cols; m++)
                {
                    elm(R, m, m) += rho;
                }

                matsolve(R, G, &wmelt);
                mattrans(wmelt, &wmel);
                matsub(nn_net->w, wmel, &nn_net->w);
                /* writemat("w", nn_net->w); */
                fejlold = fejl;
                /* beregn P */
                fejl = 0.0;
                for (k = 0; k < xl->rows; k++)
                {
                    for (m = 0; m < xl->cols; m++)
                    {
                        elm(x, m, 0) = elm(xl, k, m);
                    }
                    lpff_run(nn_net, x, &y);
                    for (m = 0; m < yl->cols; m++)
                    {
                        elm(ylv, m, 0) = elm(yl, k, m);
                    }
                    matsub(y, ylv, &err);
                    for (m = 0; m < yl->cols; m++)
                        fejl += elm(err, m, 0) * elm(err, m, 0);
                }
                rho *= 2.0;
            }
        }

    }
    killmat(&x);
    killmat(&y);
    /* killmat(&tmpin); */
    killmat(&ylv);
    killmat(&err);
    killmat(&melres3);
    killmat(&wmel);

    killmat(&out2);

    killmat(&outt);
    if (lernalgo == LW_LEARN)
    {
        killmat(&wmelt);
        killmat(&G);
        killmat(&R);
        killmat(&Rtmp);
    }
}
