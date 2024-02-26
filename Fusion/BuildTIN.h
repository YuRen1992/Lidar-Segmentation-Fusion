#pragma once
#define TIN_TRIANGLE(i) (i/3)
#define TIN_CORNER(i) (i%3)
#define TIN_INDEX(t,c) (t*3+c)
#define TIN_NEXT(c) ((c+1)%3)
#define TIN_PREV(c) ((c+2)%3)
#define POINT_BRIO_BUFFER 10000

typedef struct TINtriangle {
	float* V[3];
	int N[3];
	int next;
} TINtriangle;

class BuildTIN
{
public:
	TINtriangle* triangle_buffer = 0;
	int triangle_buffer_size = 0;
	int triangle_buffer_alloc = 0;
	int triangle_next;
	int triangle_newest;
	bool initialized = false;
	bool buffer_full = false;
	bool convex_hull_only = false;
	float* points[POINT_BRIO_BUFFER];
	bool is_little_endian = true;
	int pointer;
	BuildTIN();

	bool TINincircle(const TINtriangle* t, const float* p);
	bool TINinit(float* v0, float* v1, float* v2);
	bool TINonsegment(const float* a, const float* b, const float* p);
	void TINclean(int num, bool convex_hull = false);
	TINtriangle* TINlocate_brute(float* p);
	TINtriangle* locate_special(const float* p, TINtriangle* t);
	TINtriangle* TINlocate(float* p);
	void TINupdate(float* p, TINtriangle* t);
	bool TINinsert(float* p);
	int TINget_size();
	TINtriangle* TINget_triangle(int t);
	void TINadd(float* p);
	void TINfinish();
	void TINdestroy();
	void initialize_endianness();
	int TINinsegment(const float* a, const float* b, const float* p);
	void to_big_endian(int* value);
	void to_little_endian(int* value);

	void to_big_endian(double* value);
	void to_little_endian(double* value);
	void create_dbf_file(const char* file_name);


	~BuildTIN();
};

