#include <iostream>
#include <stdio.h>

struct ecrivain {
	char nom[50] = { 0 };
	char prenom[50] = { 0 };
	int jour, mois, naissance;
};


struct ecrivain init_ecrivain_1(void);
void afficher_ecrivaint(struct ecrivain ecriv);

int main()
{
	struct ecrivain Edmon_Rostand;
	printf_s("taille de la structure = %d\n", sizeof(struct ecrivain));

	Edmon_Rostand = init_ecrivain_1();
	afficher_ecrivaint(Edmon_Rostand);

	return 0;
}

struct ecrivain init_ecrivain_1(void)
{
	struct ecrivain res_ecrivain;

	printf_s("saisir le nom de l'ecrivain :\n");
	gets_s(res_ecrivain.nom);

	printf_s("saisir le prenom de l'ecrivain :\n");
	gets_s(res_ecrivain.prenom);

	printf_s("saisir le jour de naissance de l'ecrivain :\n");
	scanf_s("%d",&res_ecrivain.jour);

	printf_s("saisir le mois de naissance de l'ecrivain :\n");
	scanf_s("%d", &res_ecrivain.mois);

	printf_s("saisir l'annee de naissance de l'ecrivain :\n");
	scanf_s("%d", &res_ecrivain.naissance);

	return res_ecrivain;
}

void afficher_ecrivaint(struct ecrivain ecriv)
{
	printf("%s %s (%d/%d/%d)\n", ecriv.nom, ecriv.prenom, ecriv.jour,ecriv.mois,ecriv.naissance);

	return;
}






