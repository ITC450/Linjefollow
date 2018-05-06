#define elm(matx,r,c) (*(((matx)->vaerdi+*((matx)->rowoffset+c)+r)))
#define elv(veto,c) (*(veto->vaerdi+c))

typedef char stringM[40];


typedef struct
{
    int rows;
    int cols;
    float *vaerdi;
    int *rowoffset;
} matrix;

typedef struct
{
    int rows;
    float *vaerdi;
} vector;

void Error(int n);

void
SaveMat(FILE *fp, char const *pname, int mrows, int ncols, float *preal);

char
LoadMat(FILE *fp, char *pname, int *mrows, int *ncols, float **preal);

void
killmat(matrix **mat);

void
writemat(char const *navn,matrix *mat);

void
savemattxtA(char const *navn,matrix *mat,int rows, int cols);
void saveMattxt(char const *navn,matrix *mat);
void
loadMattxt(char const *navn, matrix **mat);

void
whomat(char const *navn, matrix *mat);

char
initmat(matrix **mat, int row, int col, float val);

char
getmat(FILE *fil, char const *navn, matrix **vdi);

void
savemat(FILE *fil, char const *navn, matrix *mat);

void
saveMatD(char const *navn, matrix *mat);
void
loadMatD(char const *navn, matrix **mat);


void
matadd(matrix *a, matrix *b, matrix **c);

void
matzero(matrix **a);

int matlength(matrix *a);

void
matassign(matrix *a, matrix **b);

void
matnormer(matrix * a, matrix ** b, matrix ** ext, int ori);

void
matnormerext(matrix * a, matrix ** b, matrix * ext, int ori);

void
matnormp(matrix * a, matrix ** b, matrix ** ext, int ori);

void
matnormpext(matrix * a, matrix ** b, matrix * ext, int ori);

void
matrenormp(matrix * a, matrix ** b, matrix * ext, int ori);

void
matsub(matrix *a, matrix *b, matrix **c);

void
mattrans(matrix *a, matrix **b);

void
matmulreal(matrix *a, float val, matrix **b);

void
matmul(matrix *a, matrix *b, matrix **c);

void
matdiago(matrix **a);

void
matbacksub(matrix *a, matrix **b);

void
matsolve(matrix *x, matrix *y, matrix **c);

float
l_rand(void);

float
g_rand(float my, float sigma);

char
gaussinitmat(matrix **mat, int row, int col, float my, float sigma);

char
randinitmat(matrix **mat, int row, int col, float min, float max);

char ShuffleMat(matrix *mat1, matrix *mat2, matrix *tmp);

