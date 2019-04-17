#include <assert.h>
#include "svm_common.h"

#define CELL_SIZE									(4)
#define BLOCK_SIZE									(2)
#define NBINS										(9)
#define WIN_WIDTH									(16)
#define WIN_HEIGHT									(32)

void write_tcc_file(MODEL *model,
                    char *filename)
{
	assert(model);
	assert(filename);
	
	int width = 0;
	int height = 0;
	int nBlocksX = 0;
	int blocks_width = 0;
	int y, x, z, id;
	double *ptr = NULL;
	
	FILE *fp = NULL;
	
	const char header[] = "/*\
 * =====================================================================================\n\
 *\n\
 *       Filename:  persondetectorwt.tcc\n\
 *\n\
 *    Description:  Provides person detector weights and bias.\n\
 *\n\
 *        Version:  1.0\n\
 *        Created:  Monday 28 August 2017 17:48:30 BEIJING\n\
 *       Revision:  none\n\
 *       Compiler:  MinGW\n\
 *\n\
 *         Author:  Isaac Tseng\n\
 *        Company:  ZLT\n\
 *\n\
 * =====================================================================================\n\
 */\n\n";
	
	fp = fopen(filename, "w");
	assert(fp);
	
	fputs(header, fp);
	
	fputs("float PERSON_WEIGHT_VEC[] =\n", fp);
	fputs("{\n",fp);
	printf("model->totwords %d\n", model->totwords);
	
	width = NBINS * BLOCK_SIZE;
	height = (WIN_HEIGHT / CELL_SIZE - BLOCK_SIZE + 1) * BLOCK_SIZE;
	nBlocksX = WIN_WIDTH / CELL_SIZE - BLOCK_SIZE + 1;
	blocks_width = 54;
	printf("width %d, height %d\n", width, height);
	for (z = 0; z < nBlocksX; z++) {
		for (y = 0; y < height; y++) {
			ptr = model->lin_weights + 1 + y * blocks_width + z * width;
			for (x = 0; x < width; x++) {
				fputs("(float)", fp);
				fprintf(fp, "%.15lf,\n", ptr[x]);
			}
		}
	}
	
	fputs("};\n\n", fp);
	fputs("int PERSON_WEIGHT_VEC_LENGTH = sizeof(PERSON_WEIGHT_VEC)/sizeof(float);\n", fp);
	
	fprintf(fp, "float PERSON_LINEAR_BIAS = float(%.15lf);\n", model->b);
	
	fclose(fp);
}

void write_tcc_file_v2(MODEL *model,
                       char *filename)
{
	assert(model);
	assert(filename);
	
	int width = 0;
	int height = 0;
	int nBlocksX = 0;
	int nBlocksY = 0;
	int blocks_width = 0;
	int y, x, z, id;
	double *order_weights = NULL;
	double *ptr = NULL;
	
	FILE *fp = NULL;
	
	const char header[] = "/*\
 * =====================================================================================\n\
 *\n\
 *       Filename:  persondetectorwt.tcc\n\
 *\n\
 *    Description:  Provides person detector weights and bias.\n\
 *\n\
 *        Version:  1.0\n\
 *        Created:  Monday 28 August 2017 17:48:30 BEIJING\n\
 *       Revision:  none\n\
 *       Compiler:  MinGW\n\
 *\n\
 *         Author:  Isaac Tseng\n\
 *        Company:  ZLT\n\
 *\n\
 * =====================================================================================\n\
 */\n\n";
	
	fp = fopen(filename, "w");
	assert(fp);
	
	fputs(header, fp);
	
	fputs("float PERSON_WEIGHT_VEC[] =\n", fp);
	fputs("{\n",fp);
	printf("model->totwords %d\n", model->totwords);
	
	nBlocksX = WIN_WIDTH / CELL_SIZE - BLOCK_SIZE + 1;
	nBlocksY = WIN_HEIGHT / CELL_SIZE - BLOCK_SIZE + 1;
	width = NBINS * BLOCK_SIZE * nBlocksX;
	height = BLOCK_SIZE * nBlocksY;
	
	order_weights = (double *)malloc(model->totwords * sizeof(double));
	assert(order_weights);
	
	z = 0;
	ptr = model->lin_weights + 1;
	
	for (y = 0; y < height; y++) {
		for (x = 0; x < NBINS * BLOCK_SIZE; x++) {
			order_weights[z] = ptr[y * width + x];
			z++;
		}
	}

	for (y = 0; y < height; y++) {
		for (x = NBINS * BLOCK_SIZE; x < 2 * NBINS * BLOCK_SIZE; x++) {
			order_weights[z] = ptr[y * width + x];
			z++;
		}
	}

	for (y = 0; y < height; y++) {
		for (x = 2 * NBINS * BLOCK_SIZE; x < 3 * NBINS * BLOCK_SIZE; x++) {
			order_weights[z] = ptr[y * width + x];
			z++;
		}
	}

	for (z = 0; z < model->totwords; z++) {
		fputs("(float)", fp);
		fprintf(fp, "%.15lf,\n", order_weights[z]);
	}
	
	fputs("};\n\n", fp);
	fputs("int PERSON_WEIGHT_VEC_LENGTH = sizeof(PERSON_WEIGHT_VEC)/sizeof(float);\n", fp);
	
	fprintf(fp, "float PERSON_LINEAR_BIAS = float(%.15lf);\n", model->b);
	
	fclose(fp);
	
	if (order_weights) {
		free(order_weights);
		order_weights = 0;
	}
}

int main(int argc, char* argv[])
{
	assert(argc == 2);
	
	MODEL *model;
	char *modelfile = argv[1];

	printf("Reading SVM model...");
	model=read_model(modelfile);
	printf("done\n");

	if(model->kernel_parm.kernel_type == 0) { /* linear kernel */
		/* compute weight vector */
		add_weight_vector_to_linear_model(model);
	}
	
	write_tcc_file_v2(model, (char *)("persondetectorwt.tcc"));
	
	free_model(model,1);
	
	// system("do_something.bat");
}