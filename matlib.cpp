#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "matx.h"
#include <time.h>       /* time */
#include <cstdlib>
#define MATLAB 1000

typedef struct
{
    long            type;	/* type */
    long            mrows;	/* row dimension */
    long            ncols;	/* column dimension */
    long            imagf;	/* flag indicating imag part */
    long            namlen;	/* name length (including NULL) */
}               Fmatrix;

matrix         *tmp;

int rand(void);


/************************************************************************/
/*                                                                      */
/* Retunere en TEXT-string der svare til det paag. fejl nr . <n>        */
/*                                                                      */
/* Eks:   printf("%s\n",Errstr(3));                                     */
/*                                                                      */
/************************************************************************/
char const          *
Errstr(int n)
{
    static char const  *Err[] =
    {
        "Ukendt fejl. Der er kuk i koden",
        "Kan ikke allocere plads til matrisen",
        "Matrisen findes ikke i filen",
        "<GETMAT> variabel findes ikke i filen",
        "<GETMAT> kan ikke allokere plads til matrisen",
        "<MATADD> Forskellig antal COLS i matriserne",
        "<MATADD> Forskellig antal ROWS i matriserne",
        "<MATSUB> Forskellig antal COLS i matriserne",
        "<MATSUB> Forskellig antal ROWS i matriserne",
        "<MATTRANS>  Antallet af COLS og ROWS i matriserne passer ikke sammen",
        "<MATMULREAL> Forskellig antal COLS i matriserne",
        "<MATMULREAL> Forskellig antal ROWS i matriserne",
        "<MATMUL>  Antallet af COLS og ROWS i matriserne passer ikke sammen",
        "<MATDIAGO>  Antallet af COLS og ROWS i matriserne passer ikke sammen",
        "<MATBACKSUB>  Antallet af COLS og ROWS i matriserne passer ikke sammen"
        "<MATNORMP>  Antallet af ROWS skal vaere stoerre end 2"
        "<MATNORMP>  Antallet af COLS skal vaere stoerre end 2"
    };
    return ((n < 1 || n > 16) ? Err[0] : Err[n]);
}




/************************************************************************/
/*                                                                      */
/* Udskriver en fejlmeddelse paa <stderr> som svare til fejl nr. <n>    */
/*                                                                      */
/* Eks:  if (!ok) Error(Notok_error_nr);                                */
/*                                                                      */
/************************************************************************/
void Error(int n)
{
    fprintf(stderr, "Error nr.%d : %s\n", n, Errstr(n));
    exit(1);
}


void ErrorTxt(int n, char const *txt)
{
    fprintf(stderr, "Error nr.%d : %s - %s\n", n, Errstr(n),txt);
    exit(1);
}


/************************************************************************/
/*                                                                      */
/* Gemmer et array af <double> i en standart <MATLAB>.mat fil.          */
/*                                                                      */
/* Eks:  double data[400];                                              */
/* Savemat(datfile,"kurt",20,20,data);                                  */
/*                                                                      */
/************************************************************************/
void
SaveMat(FILE *fp, char const *pname, int mrows, int ncols, float *preal)
{
    Fmatrix         x;
    int             mn;

    x.type = MATLAB;
    x.mrows = mrows;
    x.ncols = ncols;
    x.imagf = 0;
    x.namlen = strlen(pname) + 1;
    mn = x.mrows * x.ncols;

    fwrite(&x, sizeof(Fmatrix), 1, fp);
    fwrite(pname, sizeof(char), (int) x.namlen, fp);
    fwrite(preal, sizeof(float), mn, fp);
}




/************************************************************************/
/*                                                                      */
/* Laeser en <MATLAB>.mat fil.                                          */
/*                                                                      */
/* Eks:  double *data;                                                  */
/* LoadMat(datfile,navn,ant_rows,ant_cols,&data);                       */
/*                                                                      */
/************************************************************************/
char
LoadMat(FILE *fp, char *pname, int *mrows, int *ncols, float **preal)
{
    Fmatrix         x;
    int             mn, namlen;
    if (fread((char *) &x, sizeof(Fmatrix), 1, fp) != 1)
        return (1);
    *mrows = x.mrows;
    *ncols = x.ncols;
    namlen = x.namlen;
    mn = x.mrows * x.ncols;

    if (fread(pname, sizeof(char), namlen, fp) != namlen)
        return (1);
    if ((*preal = (float *) malloc(mn * sizeof(float))) == NULL)
    {
        ErrorTxt(1,"LoadMat");
    }
    if (fread(*preal, sizeof(float), mn, fp) != mn)
    {
        free(*preal);
        return (1);
    }
    return (0);
}




/************************************************************************/
/*                                                                      */
/* Frigiver det allocerede lager.                                       */
/*                                                                      */
/* Eks: matrix *m1;                                                     */
/* killmat(&m1);                                                        */
/*                                                                      */
/************************************************************************/
void
killmat(matrix **mat)
{
    free((*mat)->vaerdi);
    free((*mat)->rowoffset);
    free(*mat);
}

/************************************************************************/
/*                                                                      */
/* Udskriver en <matrix> paa skaermenen.                                */
/*                                                                      */
/* Eks: matrix *m1;                                                     */
/* writemat("vigo",m1);                                                 */
/*                                                                      */
/************************************************************************/
void
writemat(char const *navn,matrix *mat)
{
    int             i, j;
    printf("\n\n%s=\n", navn);
    for (i = 0; i < mat->rows; i++)
    {
        for (j = 0; j < mat->cols; j++)
            printf("  %10.3e", elm(mat, i, j));
        printf("\n");
    }
    printf("\n");
}

/************************************************************************/
/*                                                                      */
/* Udskriver en <matrix> paa skaermenen.                                */
/*                                                                      */
/* Eks: matrix *m1;                                                     */
/* writemat("vigo",m1);                                                 */
/*                                                                      */
/************************************************************************/
void
savemattxtA(char const *navn,matrix *mat,int rows, int cols)
{

    FILE           *infil;
    if ((infil = fopen(navn, "r+")) == NULL)
        infil = fopen(navn, "a");
    fseek( infil, 0, SEEK_SET );
    fprintf(infil,"%d", rows);
    fprintf(infil,"  %d\n", cols);
    fclose(infil);

    if ((infil = fopen(navn, "a")) == NULL)
        ErrorTxt(2,"savemattxt");

    int             i, j;
    for (i = 0; i < mat->rows; i++)
    {
        for (j = 0; j < mat->cols; j++)
        {
            fprintf(infil,"  %10.3e", elm(mat, i, j));       //printf("%d, ",j);
        }

        fprintf(infil,"\n");
    }
    fclose(infil);

}

void
saveMattxt(char const *navn,matrix *mat)
{
    FILE           *infil;
    if ((infil = fopen(navn, "w")) == NULL)
        ErrorTxt(2,"savemattxt");

    fprintf(infil,"%d", mat->rows);
    fprintf(infil,"  %d\n", mat->cols);

    for (int i = 0; i < mat->rows; i++)
    {
        for (int j = 0; j < mat->cols; j++)
        {
            fprintf(infil,"  %10.3e", elm(mat, i, j));       //printf("%d, ",j);
        }

        fprintf(infil,"\n");
    }
    fclose(infil);
}



void
loadMattxt(char const *navn, matrix **mat)
{
    FILE           *infil;
    matrix *matN;
    int rows,cols;
    float val;
    if ((infil = fopen(navn, "r")) == NULL)
        ErrorTxt(2,"loadMattxt");
    if(!fscanf(infil,"%d", &rows))  ErrorTxt(0,"loadMattxt");
    if(!fscanf(infil,"  %d\n",&cols)) ErrorTxt(0,"loadMattxt");

    initmat(mat, rows,cols, 0.0);
    int             i, j;
    for (i = 0; i < rows; i++)
    {
        for (j = 0; j < cols; j++)
        {
            if (fscanf(infil,"  %f", &val ) == EOF) break;
            elm(*mat, i, j)= val;
        }
        if (fscanf(infil,"\n")== EOF) break;
    }
    fclose(infil);
}




/************************************************************************/
/*                                                                      */
/* Udskriver col og row for <matrix> paa skaermenen.                    */
/*                                                                      */
/* Eks: matrix *m1;                                                     */
/* whomat("m1",m1);                                                   */
/*                                                                      */
/************************************************************************/
void
whomat(char const *navn, matrix *mat)
{
    printf("%s:", navn);
    printf(" size  %dx%d\n", mat->rows,mat->cols);
}




/************************************************************************/
/*                                                                      */
/* Opretter en <matrix> og initalisere alle elementer til <val>.        */
/*                                                                      */
/* Eks: matrix *mny;                                                    */
/* initmat(&mny,10,10,0.0);                                             */
/*                                                                      */
/************************************************************************/
char
initmat(matrix **mat, int row, int col, float val)
{
    int             i, j;
    if ((*mat = (matrix *) malloc(sizeof(matrix))) == NULL)
    {
        Error(1);
    }
    if (((*mat)->vaerdi = (float *) malloc(col * row * sizeof(float))) == NULL)
    {
        Error(1);
    }
    for (i = 0; i < col * row; i++)
        *((*mat)->vaerdi + i) = val;
    (*mat)->rows = row;
    (*mat)->cols = col;
    if ((((*mat)->rowoffset) = (int *) malloc(col * sizeof(int))) == NULL)
    {
        ErrorTxt(1,"initmat");
    }
    for (i = 0; i < col; i++)
    {
        *((*mat)->rowoffset + i) = i * row;
    }
    return (0);
}




/************************************************************************/
/*                                                                      */
/* Henter <matrix> med navn <navn> fra en <MATLAB>.mat fil.             */
/*                                                                      */
/* Eks: matrix *m1;                                                     */
/* getmat(datfil,"m1Name",&m1);                                           */
/*                                                                      */
/************************************************************************/
char
getmat(FILE *fil, char const *navn, matrix **vdi)
{
    float         *elmvaerdi;
    int             cols, rows, i, notfound;
    stringM          navn1;
    notfound = 1;
    i = 0;
    rewind(fil);
    while (notfound == 1)
    {
        if (0 == LoadMat(fil, navn1, &rows, &cols, &elmvaerdi))
        {
            if (strcmp(navn, navn1) == 0)
                notfound = 0;
            if (notfound == 1)
                free(elmvaerdi);
            i++;
        }
        else
        {
            ErrorTxt(3,navn);
        }
    }
    if (notfound == 0)
    {
        if ((*vdi = (matrix *) malloc(sizeof(matrix))) == NULL)
            ErrorTxt(4,"getmat");
        {
        }
        if (((*vdi)->vaerdi = (float *) malloc(sizeof(float) * rows * cols)) == NULL)
        {
            ErrorTxt(4,"getmat");
        }
        for (i = 0; i < rows * cols; i++)
        {
            *((*vdi)->vaerdi + i) = (float) *(elmvaerdi + i);
        }
        free(elmvaerdi);
        (*vdi)->rows = rows;
        (*vdi)->cols = cols;
        if ((((*vdi)->rowoffset) = (int *) malloc(cols * sizeof(int))) == NULL)
        {
            ErrorTxt(4,"getmat");
        }
        for (i = 0; i < cols; i++)
        {
            *((*vdi)->rowoffset + i) = i * rows;
        }
    }
    return (0);
}




/************************************************************************/
/*                                                                      */
/* Gemmet <matrix> paa Matlab.mat fil.                                  */
/*                                                                      */
/* Eks: matrix *m1;                                                     */
/* savemat(datafil,"m1Name",m1);                                          */
/*                                                                      */
/************************************************************************/
void
savemat(FILE *fil, char const *navn, matrix *mat)
{
    int             i;
    float         *elmvaerdi;
    if ((elmvaerdi = (float *) malloc(sizeof(float) * mat->rows * mat->cols)) == NULL)
    {
        ErrorTxt(4,"savemat");
    }
    for (i = 0; i < mat->rows * mat->cols; i++)
    {
        *(elmvaerdi + i) = (float) *((mat)->vaerdi + i);
    }
    SaveMat(fil, navn, mat->rows, mat->cols, elmvaerdi);
    free(elmvaerdi);
}

void
saveMatD(char const *navn, matrix *mat)
{
    FILE           *infil;
    if ((infil = fopen(navn, "w")) == NULL)
        ErrorTxt(2,"saveMatD");
    savemat(infil, navn, mat);
    fclose(infil);
}

void
loadMatD(char const *navn, matrix **mat)
{
    FILE           *infil;
    if ((infil = fopen(navn, "r")) == NULL)
        ErrorTxt(2,"saveMatD");
    getmat(infil, navn, mat);
    fclose(infil);
}

/************************************************************************/
/*                                                                      */
/* Addere matriserne <a> og <b> og plasere resultatet i <c>.            */
/*                                                                      */
/* Eks: matrix *m1,*m2,*sum;                                            */
/* matadd(m1,m2,&sum);                                                  */
/*                                                                      */
/************************************************************************/
void
matadd(matrix *a, matrix *b, matrix **c)
{
    int             i, j;
    if ((a->rows == b->rows) & (a->rows == (*c)->rows))
    {
        if ((a->cols == b->cols) & (a->cols == (*c)->cols))
        {

            for (i = 0; i < a->rows; i++)
                for (j = 0; j < a->cols; j++)
                    elm(*c, i, j) = elm(a, i, j) + elm(b, i, j);
        }
        else
            ErrorTxt(5,"matadd");
    }
    else
        ErrorTxt(6,"matadd");
}

/************************************************************************/
/*                                                                      */
/* Nulstiller matriserne <a>.                                           */
/*                                                                      */
/* Eks: matrix *m1;                                                     */
/* matzero(&m1);                                                        */
/*                                                                      */
/************************************************************************/
void
matzero(matrix **a)
{
    int             i, j;

    for (i = 0; i < (*a)->rows; i++)
        for (j = 0; j < (*a)->cols; j++)
            elm(*a, i, j) =0.0;
}



int matlength(matrix *a)
{
    return (a->rows * a->cols);
}

/************************************************************************/
/*                                                                      */
/* Tilskriver en matris <b> til <a>.                                    */
/*                                                                      */
/* Eks: matrix *m1,*lig_m1;                                             */
/* matassign(m1,&lig_m1);                                               */
/*                                                                      */
/************************************************************************/
void
matassign(matrix *a, matrix **b)
{
    int             i, j;
    if (a->rows == (*b)->rows)
    {
        if (a->cols == (*b)->cols)
        {

            for (i = 0; i < a->rows; i++)
                for (j = 0; j < a->cols; j++)
                    elm(*b, i, j) = elm(a, i, j);
        }
        else
            ErrorTxt(5,"matassign");
    }
    else
        ErrorTxt(6,"matassign");
}



/************************************************************************/
/*                                                                      */
/* Subtra matriserne <a> og <b> og plasere resultatet i <c>.            */
/*                                                                      */
/* Eks: matrix *m1,*m2,*dif;                                            */
/* matsub(m1,m2,&dif);                                                  */
/*                                                                      */
/************************************************************************/
void
matsub(matrix *a, matrix *b, matrix **c)
{
    int             i, j;
    if ((a->rows == b->rows) & (a->rows == (*c)->rows))
    {
        if ((a->cols == b->cols) & (a->cols == (*c)->cols))
        {

            for (i = 0; i < a->rows; i++)
                for (j = 0; j < a->cols; j++)
                    elm(*c, i, j) = elm(a, i, j) - elm(b, i, j);
        }
        else
            ErrorTxt(7,"matsub");
    }
    else
        ErrorTxt(8,"matsub");
}




/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/
void
mattrans(matrix *a, matrix **b)
{
    int             i, j;
    if (a->rows == (*b)->cols)
    {
        if (a->cols == (*b)->rows)
        {

            for (i = 0; i < a->rows; i++)
                for (j = 0; j < a->cols; j++)
                    elm(*b, j, i) = elm(a, i, j);
        }
        else
            Error(9);
    }
    else
        Error(9);
}




/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/
void
matmulreal(matrix *a, float val, matrix **b)
{
    int             i, j;
    if (a->rows == (*b)->rows)
    {
        if (a->cols == (*b)->cols)
        {

            for (i = 0; i < a->rows; i++)
                for (j = 0; j < a->cols; j++)
                    elm(*b, i, j) = elm(a, i, j) * val;
        }
        else
            Error(10);
    }
    else
        Error(11);
}




/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/
void
matmul(matrix *a, matrix *b, matrix **c)
{
    int             i, j, l;
    if (a->rows == (*c)->rows)
    {
        if ((a->cols == b->rows) & (b->cols == (*c)->cols))
        {
            for (i = 0; i < a->rows; i++)
                for (j = 0; j < b->cols; j++)
                {
                    elm(*c, i, j) = 0.0;
                    for (l = 0; l < a->cols; l++)
                        elm(*c, i, j) = elm(*c, i, j) + elm(a, i, l) * elm(b, l, j);
                }

        }
        else
            Error(12);
    }
    else
        Error(12);
}




/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/
void
matdiago(matrix **a)
{
    int             i, j, l, m;
    float           c, *tp;
    tp= (float *)malloc(sizeof(float)*((*a)->rows)+1);
    if ((*a)->rows <= (*a)->cols)
    {
        for (i = 0; i < (*a)->rows; i++)
        {
            for (j = i+1; j < (*a)->rows; j++)
            {
                if (elm(*a, i, i) == 0.0)
                {
                    for (l = i; l < (*a)->rows + 1; l++)
                    {
                        tp[l] = elm(*a, i, l);
                        elm(*a, i, l) = elm(*a, j, l);
                        elm(*a, j, l) = tp[l];
                    }
                }
                else
                {
                    if (elm(*a, i, i) != 0.0) c = -elm(*a, j, i) / elm(*a, i, i);
                    else
                    {
                        c = -elm(*a, j, i) / 0.0e-38;
                        printf("Matrix nær singulaer\n");
                    }
                    for (l = i; l < (*a)->cols; l++)
                        elm(*a, j, l) = elm(*a, j, l) + elm(*a, i, l) * c;
                    for (m = 0; m < i; m++)
                        elm(*a, j, m) = 0.0;
                }
            }
        }
    }
    else
        Error(13);
}




/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/
void
matbacksub(matrix *a, matrix **b)
{
    int             i, j;

    if ((a->rows) == ((*b)->rows) & (a->cols) == (((*b)->rows) + 1))
    {
        for (i = a->rows-1; i >= 0; i--)
        {
            elm(*b, i, 1) = elm(a, i, a->cols-1) / elm(a, i, i);
            for (j = i + 1; j < a->cols-1; j++)
            {
                elm(*b, i, 1) = elm(*b, i, 1) - elm(*b, j, 1) * elm(a, i, j) / elm(a, i, i);
            }
        }
    }
    else
        Error(14);
}




/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/
void
matsolve(matrix *x, matrix *y, matrix **c)
{
    int             i, j, l;
    initmat(&tmp, x->rows, (x->cols) + 1, 0.0);
    for (i = 0; i < x->rows; i++)
        for (j = 0; j < (x->cols); j++)
            elm(tmp, i, j) = elm(x, i, j);
    for (i = 0; i < x->rows; i++)
        elm(tmp, i, (x->cols) + 1) = elm(y, i, 1);
    matdiago(&tmp);
    /*writemat("tmp", tmp);*/
    matbacksub(tmp, c);
    killmat(&tmp);
}



/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/
float
l_rand(void)
{
    return ((float) ((double)rand() / (double)RAND_MAX));
}




/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/
float
g_rand(float my, float sigma)
{
    float           x;
    int             k;
    x = 0.0;
    for (k = 1; k <= 12; k++)
        x += l_rand();
    return (my + (x - 6.0) * sigma);
}


int tt=0;

/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/
char
gaussinitmat(matrix **mat, int row, int col, float my, float sigma)
{

    int             i, j;
    srand (time(NULL));
    if ((*mat = (matrix *) malloc(sizeof(matrix))) == NULL)
    {
        ErrorTxt(1,"gaussinitmat");
    }
    if (((*mat)->vaerdi = (float *) malloc(col * row * sizeof(float))) == NULL)
    {
        ErrorTxt(1,"gaussinitmat");
    }
    for (i = 0; i < col * row; i++)
        *((*mat)->vaerdi + i) = g_rand(my, sigma);
    (*mat)->rows = row;
    (*mat)->cols = col;
    if ((((*mat)->rowoffset) = (int *) malloc(col * sizeof(int))) == NULL)
    {
        ErrorTxt(1,"gaussinitmat");
    }
    for (i = 0; i < col; i++)
    {
        *((*mat)->rowoffset + i) = i * row;
    }
    return (0);
}




/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/
char
randinitmat(matrix **mat, int row, int col, float min, float max)
{
    int             i, j;
    //srand (time(NULL));
    if ((*mat = (matrix *) malloc(sizeof(matrix))) == NULL)
    {
        ErrorTxt(1,"randinitmat");
    }
    if (((*mat)->vaerdi = (float *) malloc(col * row * sizeof(float))) == NULL)
    {
        ErrorTxt(1,"randinitmat");
    }
    for (i = 0; i < col * row; i++)
        *((*mat)->vaerdi + i) = (max - min) * l_rand() + min;;
    (*mat)->rows = row;
    (*mat)->cols = col;
    if ((((*mat)->rowoffset) = (int *) malloc(col * sizeof(int))) == NULL)
    {
        ErrorTxt(1,"randinitmat");
    }
    for (i = 0; i < col; i++)
    {
        *((*mat)->rowoffset + i) = i * row;
    }
    return (0);
}

int randIndex(int max)
{
    return rand()% max;
}


/************************************************************************/
/*                                                                      */
/* Blanding  af en rækker i to matriser <mat1> til <mat2>.              */
/* <tmp>  er en rækkevæktor hvor tmp->cols = max(ma1->cols,mat2->cols)  */
/*                                                                      */
/*    matrix *mat1, *mat2, *tmp;                                        */
/*    ShuffleMat(mat1, mat2, tmp);                                      */
/*                                                                      */
/************************************************************************/
char ShuffleMat(matrix *mat1, matrix *mat2, matrix *tmp)
{
    for (int i=0; i<mat1->rows/2; i++)
    {
        int index1=0, index2=0;
        while (index1 == index2)
        {
            index1= randIndex(mat1->rows);
            index2= randIndex(mat1->rows);
        }

        for (int j=0; j<mat1->cols; j++)
        {
            elm(tmp,0,j)=      elm(mat1,index1,j);
            elm(mat1,index1,j)= elm(mat1,index2,j);
            elm(mat1,index2,j)= elm(tmp,0,j);
        }
        for (int j=0; j<mat2->cols; j++)
        {
            elm(tmp,0,j)=      elm(mat2,index1,j);
            elm(mat2,index1,j)= elm(mat2,index2,j);
            elm(mat2,index2,j)= elm(tmp,0,j);
        }
    }
}





/************************************************************************/
/*                                                                      */
/* Normering af en matris <b> til <a>.                                  */
/*                                                                      */
/* Eks: matrix *m1,*lig_m1. ext;                                        */
/* matmormer(m1, &lig_m1, &ext, 0);                                      */
/*                                                                      */
/************************************************************************/
void
matnormer(matrix * a, matrix ** b, matrix ** ext, int ori)
{
    int             i, j;
    if (ori == 0)
    {

        if (a->rows == (*b)->rows)
        {
            if (a->cols == (*b)->cols)
            {

                for (j = 0; j < a->rows; j++)
                {
                    elm(*ext, j, 0) = elm(a, j, 0);
                    elm(*ext, j, 1) = elm(a, j, 0);
                }

                for (i = 0; i < a->rows; i++)
                    for (j = 0; j < a->cols; j++)
                    {
                        if (elm(*ext, i, 0) < elm(a, i, j))
                            elm(*ext, i, 0) = elm(a, i, j);
                        if (elm(*ext, i, 1) > elm(a, i, j))
                            elm(*ext, i, 1) = elm(a, i, j);
                    }
                for (i = 0; i < a->rows; i++)
                    for (j = 0; j < a->cols; j++)
                    {
                        elm(*b, i, j) = (elm(a, i, j) - elm(*ext, i, 1)) / (1.25 * (elm(*ext, i, 0) - elm(*ext, i, 1))) + 0.1;
                    }
            }
            else
                Error(5);
        }
        else
            Error(6);


    }
    else
    {

        if (a->rows == (*b)->rows)
        {
            if (a->cols == (*b)->cols)
            {

                for (j = 0; j < a->cols; j++)
                {
                    elm(*ext, j, 0) = elm(a, 0, j);
                    elm(*ext, j, 1) = elm(a, 0, j);
                }

                for (j = 0; j < a->cols; j++)
                    for (i = 0; i < a->rows; i++)
                    {
                        if (elm(*ext, j, 0) < elm(a, i, j))
                            elm(*ext, j, 0) = elm(a, i, j);
                        if (elm(*ext, j, 1) > elm(a, i, j))
                            elm(*ext, j, 1) = elm(a, i, j);
                    }
                for (i = 0; i < a->rows; i++)
                    for (j = 0; j < a->cols; j++)
                    {
                        elm(*b, i, j) = (elm(a, i, j) - elm(*ext, j, 1)) / (1.25 * (elm(*ext, j, 0) - elm(*ext, j, 1))) + 0.1;
                    }
            }
            else
                Error(5);
        }
        else
            Error(6);


    }

}


/************************************************************************/
/*                                                                      */
/* Normering af en matris <b> til <a> med extrenun.                     */
/*                                                                      */
/* Eks: matrix *m1,*lig_m1. *ext;                                        */
/* matmormer(m1, &lig_m1, ext, 0);                                     */
/*                                                                      */
/************************************************************************/
void
matnormerext(matrix * a, matrix ** b, matrix * ext, int ori)
{
    int             i, j;
    if (ori == 0)
    {

        if (a->rows == (*b)->rows)
        {
            if (a->cols == (*b)->cols)
            {

                for (i = 0; i < a->rows; i++)
                    for (j = 0; j < a->cols; j++)
                    {
                        elm(*b, i, j) = (elm(a, i, j) - elm(ext, i, 1)) / (1.25 * (elm(ext, i, 0) - elm(ext, i, 1))) + 0.1;
                    }
            }
            else
                Error(5);
        }
        else
            Error(6);


    }
    else
    {

        if (a->rows == (*b)->rows)
        {
            if (a->cols == (*b)->cols)
            {

                for (i = 0; i < a->rows; i++)
                    for (j = 0; j < a->cols; j++)
                    {
                        elm(*b, i, j) = (elm(a, i, j) - elm(ext, j, 1)) / (1.25 * (elm(ext, j, 0) - elm(ext, j, 1))) + 0.1;
                    }
            }
            else
                Error(5);
        }
        else
            Error(6);


    }

}



/************************************************************************/
/*                                                                      */
/* Normering af en matris <m1> til <lig_m1>.                            */
/* <lig_m1> bliver normeret til en middelværdi på nul og en             */
/* standartafvigelse på 0.5                                             */
/* <ext> er en matris indeholdende oprindelig middelværdi og Std.afv.   */
/* ori:  0 - Rækkenormering.                                            */
/*       1 - Søjlenormering.                                            */
/*                                                                      */
/* Eks: matrix *m1,*lig_m1. ext;                                        */
/* matmormp(m1, &lig_m1, &ext, 0);                                      */
/*                                                                      */
/************************************************************************/
void
matnormp(matrix * a, matrix ** b, matrix ** ext, int ori)
{
    int             i, j;
    if (ori == 0)
    {
        if (a->cols < 2) Error(15);
        if (a->rows == (*b)->rows)
        {
            if (a->cols == (*b)->cols)
            {

                for (j = 0; j < a->rows; j++)
                {
                    elm(*ext, j, 0) = 0.0;
                    elm(*ext, j, 1) = 0.0;
                }

                for (i = 0; i < a->rows; i++)
                    for (j = 0; j < a->cols; j++)
                    {
                        elm(*ext, i, 0) += elm(a, i, j);
                    }
                for (i = 0; i < a->rows; i++)
                {
                    elm(*ext, i, 0) += elm(*ext, i, 0)/(float)a->cols;
                }


                for (i = 0; i < a->rows; i++)
                    for (j = 0; j < a->cols; j++)
                    {
                        elm(*ext, i, 1) += (elm(a, i, j)-elm(*ext, i, 0))*(elm(a, i, j)-elm(*ext, i, 0));
                    }

                for (i = 0; i < a->rows; i++)
                {
                    elm(*ext, i, 1) += elm(*ext, i, 1)/((float)a->cols-1);
                }


                for (i = 0; i < a->rows; i++)
                    for (j = 0; j < a->cols; j++)
                    {
                        elm(*b, i, j) = (elm(a, i, j) - elm(*ext, i, 0)) / (2*elm(*ext,i,1));
                    }

            }
            else
                Error(5);
        }
        else
            Error(6);


    }
    else
    {

        if (a->rows < 2) Error(16);
        if (a->rows == (*b)->rows)
        {
            if (a->cols == (*b)->cols)
            {

                for (j = 0; j < a->cols; j++)
                {
                    elm(*ext, j, 0) = 0.0;
                    elm(*ext, j, 1) = 0.0;
                }

                for (j = 0; j < a->cols; j++)
                    for (i = 0; i < a->rows; i++)
                    {
                        elm(*ext, j, 0) += elm(a, i, j);
                    }
                for (j = 0; j < a->cols; j++)
                {
                    elm(*ext, j, 0) = elm(*ext, j, 0)/(float)a->rows;
                }


                for (j = 0; j < a->cols; j++)
                    for (i = 0; i < a->rows; i++)
                    {
                        elm(*ext, j, 1) += (elm(a, i, j)-elm(*ext, j, 0))*(elm(a, i, j)-elm(*ext, j, 0));
                        //printf("j: %d elm(a, i, j)) %f : elm(*ext, j, 1) %f\n",j,elm(a, i, j),elm(*ext, j, 1));
                    }

                for (j = 0; j < a->cols; j++)
                {
                    elm(*ext, j, 1) = sqrt(elm(*ext, j, 1)/((float)a->rows));
                    //printf("j: %d elm(*ext, j, 1) %f : elm(*ext, j, 1)/((float)a->rows)-1 %f\n",j,elm(*ext, j, 1),elm(*ext, j, 1)/((float)a->rows)-1);
                }

                for (i = 0; i < a->rows; i++)
                    for (j = 0; j < a->cols; j++)
                    {
                        elm(*b, i, j) = (elm(a, i, j) - elm(*ext, j, 0)) / (2*elm(*ext,j,1));
                    }
            }
            else
                Error(5);
        }
        else
            Error(6);


    }

}

/************************************************************************/
/*                                                                      */
/* Normering af en matris <b> til <a>.                                  */
/*                                                                      */
/* Eks: matrix *m1,*lig_m1. ext;                                        */
/* matmormpext(m1, &lig_m1, &ext, 0);                                   */
/*                                                                      */
/************************************************************************/
void
matnormpext(matrix * a, matrix ** b, matrix * ext, int ori)
{
    int             i, j;
    if (ori == 0)
    {

        if (a->rows == (*b)->rows)
        {
            if (a->cols == (*b)->cols)
            {

                for (i = 0; i < a->rows; i++)
                    for (j = 0; j < a->cols; j++)
                    {
                        elm(*b, i, j) = (elm(a, i, j) - elm(ext, i, 0)) / (2*elm(ext,i,1));
                    }

            }
            else
                Error(5);
        }
        else
            Error(6);


    }
    else
    {

        if (a->rows == (*b)->rows)
        {
            if (a->cols == (*b)->cols)
            {

                for (i = 0; i < a->rows; i++)
                    for (j = 0; j < a->cols; j++)
                    {
                        elm(*b, i, j) = (elm(a, i, j) - elm(ext, j, 0)) / (2*elm(ext,j,1));
                    }
            }
            else
                Error(5);
        }
        else
            Error(6);


    }

}

/************************************************************************/
/*                                                                      */
/* Normering af en matris <b> til <a>.                                  */
/*                                                                      */
/* Eks: matrix *m1,*lig_m1. ext;                                        */
/* matrenormpext(m1, &lig_m1, &ext, 0);                                 */
/*                                                                      */
/************************************************************************/
void
matrenormp(matrix * a, matrix ** b, matrix * ext, int ori)
{
    int             i, j;
    if (ori == 0)
    {

        if (a->rows == (*b)->rows)
        {
            if (a->cols == (*b)->cols)
            {

                for (i = 0; i < a->rows; i++)
                    for (j = 0; j < a->cols; j++)
                    {
                        elm(*b, i, j) = (elm(a, i, j) * 2 * elm(ext,i,2)) + elm(ext, j, 1);
                    }

            }
            else
                Error(5);
        }
        else
            Error(6);


    }
    else
    {

        if (a->rows == (*b)->rows)
        {
            if (a->cols == (*b)->cols)
            {

                for (i = 0; i < a->rows; i++)
                    for (j = 0; j < a->cols; j++)
                    {
                        elm(*b, i, j) = (elm(a, i, j) * 2 * elm(ext,j,2)) + elm(ext, j, 1);
                    }
            }
            else
                Error(5);
        }
        else
            Error(6);


    }

}

