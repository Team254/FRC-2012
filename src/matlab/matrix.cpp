#include <stdlib.h>
#include <stdio.h>
#include "stdarg.h"

#include "matrix.h"

#ifdef __cplusplus
extern "C" {
#endif


struct matrix *init_matrix(int n_rows, int n_cols){
  struct matrix *mat;
  int i;
  mat = (struct matrix *)malloc(sizeof(struct matrix));
  mat->width = n_cols;
  mat->height = n_rows;
  mat->data = (double *)malloc(sizeof(double) * n_rows * n_cols);
  for(i = 0; i < n_rows * n_cols; i++)
    (mat->data)[i] = 0.0;
  return(mat);
}

double matrix_get(struct matrix *mat, int y, int x){
  if (x >= mat->width || y >= mat->height){
    fprintf(stderr,"out of bounds(%d,%d)\n", x, y);
    return(0.0);
  }
  return(mat->data[x + mat->width * y]);
}
int matrix_set(struct matrix *mat, int y, int x, double val){
  if (x >= mat->width || y >= mat->height){
    fprintf(stderr,"out of bounds(%d,%d)=%g\n", x, y, val);
    return(0);
  }
  mat->data[x + mat->width * y] = val;
  return(1);
}
struct matrix *make_eye_matrix(int s){
  struct matrix *mat = init_matrix(s, s);
  int i;
  int comp = s * s;
  int incr = s + 1;
  double *data = mat->data;
  for(i = 0; i < comp; i += incr){
    data[i] = 1.0;
  }
  return(mat);
}
int matrix_check_same(struct matrix *dest,
    struct matrix *mat1, struct matrix *mat2){
  if (mat1->width != mat2->width ||
      mat1->height != mat2->height ||
      mat1->height != dest->height ||
      mat1->width  != dest->width){
    fprintf(stderr,
      "wrong_size dest={%d,%d} mat1={%d,%d} mat2={%d,%d}\n",
      dest->width, dest->height,
      mat1->width, mat1->height,
      mat2->width, mat2->height);
    return(0);
  }
  return(1);
}
int matrix_minus(struct matrix *dest,
    struct matrix *mat1, struct matrix *mat2){
  if (!matrix_check_same(dest, mat1, mat2))
    return(0);
  double *data1 = mat1->data;
  double *datadest = dest->data;
  double *data2 = mat2->data;
  int comp = mat1->width * mat1->height;
  int i;
  for(i = 0; i < comp; i++){
    datadest[i] = data1[i] - data2[i];
  }
  return(1);
}
int matrix_add(struct matrix *dest,
    struct matrix *mat1, struct matrix *mat2){
  if (!matrix_check_same(dest, mat1, mat2))
    return(0);
  double *data1 = mat1->data;
  double *datadest = dest->data;
  double *data2 = mat2->data;
  int comp = mat1->width * mat1->height;
  int i;
  for(i = 0; i < comp; i++){
    datadest[i] = data1[i] + data2[i];
  }
  return(1);

}
int matrix_mult(struct matrix *dest,
    struct matrix *mat1, struct matrix *mat2){
  int dest_height = mat1->height;
  int dest_width = mat2->width;
  int dw_mis = dest->width != dest_width;
  int dh_mis = dest->height != dest_height;
  int p_mis  = mat1->width != mat2->height;
  if (dw_mis || dh_mis || p_mis){
    fprintf(stderr,"differences shown:");
    if (dw_mis) fprintf(stderr, " dest={%d,", dest->width);
    else fprintf(stderr, " dest={__,");
    if (dh_mis) fprintf(stderr, "%d}", dest->height);
    else fprintf(stderr, "__}");
    if (p_mis) fprintf(stderr, " mat1={%d", mat1->width);
    else fprintf(stderr, " mat1={__");
    if (dh_mis) fprintf(stderr, ",%d}", mat1->height);
    else fprintf(stderr, ",__}");
    if (dw_mis) fprintf(stderr, " mat1={%d", mat2->width);
    else fprintf(stderr, " mat2={__");
    if (p_mis) fprintf(stderr, ",%d}\n", mat2->height);
    else fprintf(stderr, ",__}\n");
    return(0);
  }
  double *data1 = mat1->data;
  double *datadest = dest->data;
  double *data2 = mat2->data;
  int i;
  int j;
  int p_max = mat1->width;
  int width1 = mat1->width;
  int width2 = mat2->width;
  int p;
  for(i = 0; i < dest_width; i++){
    for(j = 0; j < dest_height; j++){
      double tmp = 0.0;
      for(p = 0; p < p_max; p++){
        tmp += data2[i + width2 * p] *
          data1[p + width1 * j];
      }
      datadest[i + dest_width * j] = tmp;
    }
  }
  return(1);
}
int matrix_scale(struct matrix *dest,double value,struct matrix *mat){
  double *data_dest = dest->data;
  double *data_src = mat->data;
  int i;
  int comp = mat->width * mat->height;
  if (comp != dest->width * dest->height){
    fprintf(stderr, "Invalid Dims\n");
    return(0);
  }
  for(i = 0; i < comp; i++){
    data_dest[i] = data_src[i] * value;
  }
  return(1);
}
int print_matrix(struct matrix *mat){
  int i;
  int j;
  int w = mat->width;
  int h = mat->height;
  for(j = 0; j < h; j++){
    for(i = 0; i < w; i++){
      printf("%g ", (mat->data)[i + j * w]);
    }
    printf("\n");
  }
  return(1);
}
int write_matrix(int rows, int cols,...){

}
void flash_matrix(struct matrix *mat,...){
  int count = mat->width * mat->height;
  va_list args;
  int i;
  va_start(args, mat);
  for(i = 0; i < count; i++){
    mat->data[i] = va_arg(args,double);
  }
  va_end(args);
}
void free_matrix(struct matrix *mat){
  free(mat->data);
  free(mat);
}

#ifdef __cplusplus
}
#endif
