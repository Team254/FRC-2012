/**
  * This is a basic matrix library written by Parker Schuh which was chosen
  * because we didn't want to write our own library. Unfortunately, this
  * code is all written in C which makes performing multiple matrix
  * operations quite bothersome. This should be updated to use a C++ style
  * library at some point, but during the season it worked and we didn't
  * feel the need to change it.
  */

#ifndef _MATRIX_H_
#define _MATRIX_H_
#ifdef __cplusplus
extern "C" {
#endif
struct matrix{
	int width;
	int height;
	double *data;
};
struct matrix *init_matrix(int n_rows,int n_cols);
double matrix_get(struct matrix *mat,int y,int x);
int matrix_set(struct matrix *mat,int y,int x,double val);
struct matrix *make_eye_matrix(int s);
int matrix_minus(struct matrix *dest,
		struct matrix *mat1, struct matrix *mat2);
int matrix_add(struct matrix *dest,
		struct matrix *mat1, struct matrix *mat2);
int matrix_mult(struct matrix *dest,
		struct matrix *mat1, struct matrix *mat2);
int matrix_scale(struct matrix *dest,
		double value, struct matrix *mat);
int print_matrix(struct matrix *mat);
void flash_matrix(struct matrix *mat,...);
void free_matrix(struct matrix *mat);

#ifdef __cplusplus
}
#endif
#endif
