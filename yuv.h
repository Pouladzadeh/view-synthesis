/***********************************************************

             YUV image input/output functions

 ***********************************************************/
  #include <stdio.h>
  #include <stdlib.h>
  #include <math.h>
  #include <string.h>

/*=======================================================
  -------------------------------------------------------
  Structure for an image size
  =======================================================*/
typedef struct _point {
  int	x, y;
} Point;


/*=======================================================
  -------------------------------------------------------
  Structure for an image store memory
  =======================================================*/
typedef struct _image {
  int		type;	    /* Type of iamge  1: gray, 3: color, 2: yuv(4:2:0) */
  Point		size;	    /* Size of iamge */
  unsigned char *data ;	/* Image data */
} Image;


/*=======================================================
  -------------------------------------------------------
  Generate an image store memory
  -------------------------------------------------------
  [Argument]
      int x: x size (width of image)
      int y: y size (height of image)
	  int m: channel number
  -------------------------------------------------------
  [Return Value]
      Image *: pointer to image memory structure
  =======================================================*/
Image *new_image( int x, int y, int m)
{
  Image	*a;

  a = (Image *)calloc(1, sizeof( Image ) );
  if( a == NULL ) exit( -1 );
  a->type = m;
  a->size.x = x;
  a->size.y = y;
  a->data = (unsigned char *)calloc(m * x * y, sizeof(unsigned char)) ;
  if( a->data == NULL ) exit( -1 );

  return( a );
}


/*=======================================================
  -------------------------------------------------------
  Release an image store memory
  -------------------------------------------------------
  [Argument]
      Image *a: pointer to image memory structure
  -------------------------------------------------------
  [Return Value]
      No
  =======================================================*/
void free_image( Image *a )
{
  if( a != NULL ){
    if( a->data != NULL ) free( a->data );
    free( a );
  }
}
  



/*=======================================================
  -------------------------------------------------------
	YUV image read 
  -------------------------------------------------------
  [Argument]
     char	*file     Input file name
  -------------------------------------------------------
  [Return Value]
     Image	*     Pointer to image memory structure
  =======================================================*/
Image *yuvread( char *file, int width, int height )
{
    unsigned char	*buff,*b1; 
	unsigned char	*b2; // unsigned char
    FILE     		*fp;
    int      		i, j ;
    Image		*img;

    if ((fp=fopen(file,"rb")) == NULL) exit( -1 );
     
//  fprintf(stderr, "Reading YUV format, %d x %d image : %s\n", x, y, file);

    img = new_image(width, height, 2);
//	IplImage* img = cvCreateImage(cvSize(width, height), 8, 3);

    buff = (unsigned char *)calloc( 3 * width * height /2, sizeof(unsigned char) ); //unsigned char
    if( buff == NULL ) exit( -1 );

    fread(buff, 1, 3 * width * height / 2, fp);
    b2 = img->data;
    b1 = buff;
    for (i=0; i < 3*(height/2); i++){
      for (j=0; j < width; j++){
		  *b2++ = *b1++;
      }
    }
		
    fclose(fp);
    free(buff);

    return(img);
}


/*=======================================================
  -------------------------------------------------------
  Conversion of YUV(YCbCr) 4:2:0 format into BGR format
  -------------------------------------------------------
  [Argument]
    Image *im_yuv: pointer to image memory structure for YUV
    Image *im:     pointer to image memory structure for BGR
  -------------------------------------------------------
  [Return Value]
    No
  =======================================================*/
static void YUV2BGR(Image *im_yuv, IplImage *im_bgr)
{
	int ir, ig, ib;
	int width, height;
	width = im_bgr->width;
	height = im_bgr->height;

	for(int i = 0 ; i < height; i++){
      for(int j = 0 ; j < width; j++){
		  ib = (im_yuv->data[i * width + j] + 1.772*(im_yuv->data[height * width + (i/2) * (width/2) + (j/2)]-127)+0.5);
		  if(ib>255)
			  im_bgr->imageData[(i * width + j) * 3] = 255;
		  else if (ib<0)
			  im_bgr->imageData[(i * width + j) * 3] = 0;
		  else
			  im_bgr->imageData[(i * width + j) * 3] = ib;

		  ig = (im_yuv->data[i * width + j] - 0.344*(im_yuv->data[height * width + (i/2) * (width/2) + (j/2)]-127) - 0.714*(im_yuv->data[height * width + (height/2) * (width/2) + (i/2) * (width/2) + (j/2)]-127)+0.5);
		  if(ig>255)
			  im_bgr->imageData[(i * width + j) * 3 + 1] = 255;
		  else if (ig<0)
			  im_bgr->imageData[(i * width + j) * 3 + 1] = 0;
		  else
			  im_bgr->imageData[(i * width + j) * 3 + 1] = ig;

		  ir = (im_yuv->data[i * width + j] + 1.402*(im_yuv->data[height * width + (height/2) * (width/2) + (i/2) * (width/2) + (j/2)]-127)+0.5);
		  if (ir>255)
			  im_bgr->imageData[(i * width + j) * 3 + 2] = 255;
		  else if (ir<0)
			  im_bgr->imageData[(i * width + j) * 3 + 2] = 0;
		  else
			  im_bgr->imageData[(i * width + j) * 3 + 2] = ir;
      }
    }
}


/*=======================================================
  -------------------------------------------------------
  Memory allocation
  -------------------------------------------------------
  [Argument]
    int width: width of image
    int height: height of image
  -------------------------------------------------------
  [Return Value]
    unsigned char**     Pointer to memory structure
  =======================================================*/

unsigned char **ByteAlloc(int width, int height)
{
	unsigned char *p, **pp ;
	pp = (unsigned char **)malloc (sizeof(unsigned char *) * height) ;
	pp[0] = (unsigned char *)malloc(sizeof(unsigned char) * width * height) ;
	p = pp[0] ;
	for (int i = 1; i < height; i++)
	{
		p += width ;
		pp[i] = p ;
	}
	return pp ;
}


/*=======================================================
  -------------------------------------------------------
  Conversion of BGR format into YUV (4:2:0) format
  -------------------------------------------------------
  Argument
    Image *im:     pointer to image memory structure for BGR
    Image *im_yuv: pointer to image memory structure for YUV
  -------------------------------------------------------
  [Return Value]
    No
  =======================================================*/

static void BGR2YUV(IplImage *im_bgr, unsigned char **im_y, unsigned char **im_u, unsigned char **im_v)
{

	unsigned char fr, fg, fb ;
	int width, height;
	width = im_bgr->width;
	height = im_bgr->height;

	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			fb = (im_bgr->imageData[(i * width + j)*3]) ;	//B
			fg = (im_bgr->imageData[(i * width + j)*3+1]) ;	//G
			fr = (im_bgr->imageData[(i * width + j)*3+2]) ;	//R

				im_y[i][j] = (unsigned char)(0.299 * fr + 0.587 * fg + 0.114 * fb + 0.5) ;	        //Y
			if(i%2 == 0 && j%2 == 0){
				im_u[i/2][j/2] = (unsigned char)((-0.169 * fr - 0.331 * fg + 0.500 * fb) + 127.5) ;	//U
				im_v[i/2][j/2] = (unsigned char)((0.500 * fr - 0.419 * fg - 0.081 * fb) + 127.5) ;	//V
			}
		}
	}
}

