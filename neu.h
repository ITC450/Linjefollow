#define SIGMOID(x) 1.0/(1.0+exp(-x))
#define SIGMOIDdif(x) x*(1-x)

typedef struct
{
    matrix *inw,*udw,*inoff,*udoff, *udwt, *indwt, *inwmel, *udwmel;
    matrix *fejl1, *fejl2, *melres1, *melres2, *melres3;
    matrix *out0, *out1;
} mlp_net;

#define BPE_LEARN 0
#define LW_LEARN 1

typedef struct
{
    matrix *w, *out, *ffout, *antff, *ext, *b, *del;
} lpff_net;

typedef struct
{
    matrix *w,*lampda,*y;
    float rinit;
} sph_net;

void Sphere_init(sph_net **nn_net, float rinit);
void Sphere_train(sph_net *nn_net, matrix * xl, matrix * yl);
void Sphere_run(sph_net *nn_net, matrix * x, matrix ** y, int *conf, int *antC);

void rce_train(matrix * xl, matrix * yl, float rinit, matrix ** w, matrix ** lampda, matrix **y);
void rce_run(matrix * w, matrix * lampda, matrix *y, matrix *x, matrix **nnoutput, int *conf, int *antC);

void matrenormp(matrix * a, matrix ** b, matrix * ext, int ori);

void
ligeinit_bpe(mlp_net ** netvark, int antinput, int antneuroner, int antoutput, float min, float max);
void
gaussinit_bpe(mlp_net ** netvark, int antinput, int antneuroner, int antoutput, float my, float sigma);
void
mlp_train(mlp_net * nn_net, matrix * xl, matrix * yl, int ant_itt, float alfa, char const *ErrFilName);
void
bpe_forward(matrix * netind, mlp_net * netvark, matrix ** netud);
void
load_bpenet(char const *filnavn, mlp_net ** netvark, int *antinput, int *antneuroner, int *antoutput);
void
save_bpenet(char const *filnavn, mlp_net * netvark);
void
kill_bpenet(mlp_net ** netvark);

void
write_bpenet(mlp_net *netvark);
void
parzen(matrix * xl, matrix * yl, float h, matrix * x, matrix ** netud);
void
save_prazen(char const *filnavn, matrix * xl, matrix * yl, float h);
void
load_prazen(char const *filnavn, matrix ** xl, matrix ** yl, float *h);
void
Sphere_train(sph_net * nn_net, matrix * xl, matrix * yl);
void
Sphere_run(sph_net * nn_net, matrix * x, matrix ** y, int *conf, int *antC);
void
load_sphnet(char const *filnavn, sph_net ** netvark);
void
save_sphnet(char const *filnavn, sph_net * netvark);
void
kill_sphnet(sph_net ** netvark);


void
lpff_init(lpff_net ** nn_net, matrix * antff, matrix * xl, int antoutput, float kl);
void
lpff_train(lpff_net * nn_net, matrix * xl, matrix * yl, char lernalgo, int antit, float alpha);
void
lpff_run(lpff_net * nn_net, matrix * x, matrix ** y);
void
save_lpffnet(char const *filnavn, lpff_net * netvark);
void
load_lpffnet(char const *filnavn, lpff_net ** netvark);
void
kill_lpffnet(lpff_net ** netvark);


