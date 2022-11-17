#include <iostream>
#include <stdio.h>
#include <math.h>

struct Point {

	int x, y;
};

struct rectangle {
	struct Point sommet_inf;
	struct Point sommet_sup;
};

void afficher_point_pt(const char *, struct Point * );
double calcule_distance_pt(struct Point * , struct Point * );
void afficher_distance_pt(double);
void calcule_perimetre_pt(struct rectangle *);
void calcule_surface_pt(struct rectangle * );


int main()
{
	struct Point A = { 3,5 }, B;
	struct Point * Pt_A, *Pt_B;
	struct rectangle Mon_rectangle;
	struct rectangle * Pt_rectangle;


	Pt_A = &A;
	Pt_B = &B;

	Pt_B->x = 4;
	Pt_B->y = 7;

	Pt_rectangle = &Mon_rectangle;
	Pt_rectangle->sommet_inf = A;
	Pt_rectangle->sommet_sup = B;


	afficher_point_pt("A", Pt_A);
	afficher_point_pt("B", Pt_B);

	afficher_distance_pt(calcule_distance_pt(Pt_A, Pt_B));
	calcule_perimetre_pt(Pt_rectangle);
	calcule_surface_pt(Pt_rectangle);

	return(1);
}

void afficher_point_pt(const char * nom, struct Point *  pt_Mon_Point)
{
	printf("%s = (%d,%d)\n", nom, pt_Mon_Point->x, pt_Mon_Point->y);

	return;
}

double calcule_distance_pt(struct Point  * Point_A, struct Point *  Point_B)
{
	int xA, yA, xB, yB;
	double distance;

	xA = Point_A->x;
	yA = Point_A->y;

	xB = Point_B->x;
	yB = Point_B->y;

	distance = sqrt(pow(abs(xA - xB), 2) + pow(abs(yA - yB), 2));

	return distance;
}

void afficher_distance_pt(double distance)
{
	printf("la distance est  : %4.2f\n", distance);
}


void calcule_perimetre_pt(struct rectangle  * pt_Mon_rectangle)
{
	int perimetre;
	int Dx, Dy;

	Dx = abs(pt_Mon_rectangle->sommet_inf.x - pt_Mon_rectangle->sommet_sup.x);
	Dy = abs(pt_Mon_rectangle->sommet_inf.y - pt_Mon_rectangle->sommet_sup.y);

	perimetre = 2 * (Dx + Dy);
	printf("perimetre  = %d\n", perimetre);
}


void calcule_surface_pt(struct rectangle *  Mon_rectangle)
{
	int surface;
	int Dx, Dy;

	Dx = abs(Mon_rectangle->sommet_inf.x - Mon_rectangle->sommet_sup.x);
	Dy = abs(Mon_rectangle->sommet_inf.y - Mon_rectangle->sommet_sup.y);

	surface = Dx * Dy;
	printf("surface  = %d\n", surface);
}