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

void afficher_point(const char *, struct Point Mon_Point);
double calcule_distance(struct Point, struct Point);
void afficher_distance(double);

void calcule_perimetre(struct rectangle);
void calcule_surface(struct rectangle);


int main()
{
	struct Point A = { 3,5 }, B;
	struct rectangle Mon_rectangle;

	B.x = 4;
	B.y = 7;

	afficher_point("A", A);
	afficher_point("B", B);

	afficher_distance(calcule_distance(A,B));

	Mon_rectangle.sommet_inf = A;
	Mon_rectangle.sommet_sup = B;

	calcule_perimetre(Mon_rectangle);
	calcule_surface(Mon_rectangle);

	return(1);
}

void afficher_point(const char * nom, struct Point Mon_Point)
{
	printf("%s = (%d,%d)\n", nom, Mon_Point.x,Mon_Point.y);

	return;
}

double calcule_distance(struct Point Point_A, struct Point Point_B)
{
	int xA, yA, xB, yB;
	double distance;

	xA = Point_A.x;
	yA = Point_A.y;

	xB = Point_B.x;
	yB = Point_B.y;

	distance = sqrt(pow(abs(xA - xB), 2) + pow(abs(yA - yB), 2));

	return distance;
}

void afficher_distance(double distance)
{
	printf("la distance est  : %4.2f\n  ", distance);
}


void calcule_perimetre(struct rectangle Mon_rectangle)
{
	int perimetre;
	int Dx, Dy;

	Dx = abs(Mon_rectangle.sommet_inf.x - Mon_rectangle.sommet_sup.x);
	Dy = abs(Mon_rectangle.sommet_inf.y - Mon_rectangle.sommet_sup.y);

	perimetre = 2 * (Dx + Dy);
	printf("perimetre  = %d\n", perimetre);
}


void calcule_surface(struct rectangle Mon_rectangle)
{
	int surface;
	int Dx, Dy;

	Dx = abs(Mon_rectangle.sommet_inf.x - Mon_rectangle.sommet_sup.x);
	Dy = abs(Mon_rectangle.sommet_inf.y - Mon_rectangle.sommet_sup.y);

	surface= Dx * Dy;
	printf("surface  = %d\n", surface);
}