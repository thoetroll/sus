#include <windows.h>
#include <stdio.h>


struct noeud { char * nom; char * prenom; int jour; int mois; int annee; noeud * suivant; };
typedef struct noeud NOEUD;

struct entete { int longueur; NOEUD * premier; NOEUD * dernier; };
typedef struct entete ENTETE;

#define MAX_LENGTH 50

int menu_liste(void);
ENTETE * creer_entete(void);
void insere_debut(ENTETE *);
NOEUD * creer_noeud(void);
void init_ecrivain(NOEUD *);
void insere_fin(ENTETE *);
void supprime_elt(ENTETE *);
void RAZ_noeud(NOEUD *);
void affiche_list(ENTETE *);
void affiche_noeud(NOEUD *);
void RAZ_memoire(ENTETE *);
void enregistrer_list(ENTETE *);
void memo_noeud(NOEUD *, FILE*);
ENTETE * recup_list(void);
NOEUD * charger_noeud(FILE *);
bool fin_fichier(FILE*);
void charger_fin(ENTETE *, NOEUD*);



int main()
{
	int choix = 1;

	ENTETE * Pteur_entete;
	Pteur_entete = creer_entete();

	while (choix > 0)
	{
		choix = menu_liste();
		switch (choix)
		{
		case 0:
		{
			printf("\nCIAO\n");
			RAZ_memoire(Pteur_entete);
			break;
		}
		case 1:
		{
			insere_debut(Pteur_entete);
			break;
		}
		case 2:
		{
			insere_fin(Pteur_entete);
			break;
		}
		case 3:
		{
			supprime_elt(Pteur_entete);
			break;
		}
		case 4:
		{
			affiche_list(Pteur_entete);
			break;
		}
		case 5:
		{
	
		    enregistrer_list(Pteur_entete);
			break;
		}
		case 6:
		{
			RAZ_memoire(Pteur_entete);

			Pteur_entete= recup_list();
			break;
		}
		default:
		{
			printf("\nMAUVAIS CHOIX\n");
			break;
		}
		}
	}


}

int menu_liste()
{
	int choix = 0;
	system("cls");
	printf("Sortir -> 0");
	printf("\nAjouter en tete -> 1");
	printf("\nAjouter en queue -> 2");
	printf("\nSupprimer ELT -> 3");
	printf("\nAfficher liste -> 4");
	printf("\nEnregistre liste -> 5\n");

	printf("\nVotre choix ->");
	scanf_s("%d", &choix);
	return choix;

}

ENTETE * creer_entete(void)
{
	ENTETE * nouveau;

	if (nouveau = (ENTETE *)malloc(sizeof(ENTETE)))
	{
		nouveau->longueur = 0;
		nouveau->premier = NULL;
		nouveau->dernier = NULL;
	}
	else
	{
		printf("Probleme entete");
	}
	return(nouveau);
}

NOEUD * creer_noeud(void)
{
	NOEUD * nouveau;

	if (nouveau = (NOEUD *)malloc(sizeof(NOEUD)))
	{
		init_ecrivain(nouveau);
		nouveau->suivant = NULL;
	}
	else
	{
		printf("Probleme noeud");
	}
	return(nouveau);
}

void init_ecrivain(NOEUD * ecrivain)
{
	char * Pteur_nom, *Pteur_prenom;

	Pteur_nom = (char *)malloc(MAX_LENGTH * sizeof(char));
	Pteur_prenom = (char *)malloc(MAX_LENGTH * sizeof(char));


	system("cls");
	rewind(stdin);

	printf("nom ecrivain ->");
	fgets(Pteur_nom, MAX_LENGTH, stdin);
	ecrivain->nom = Pteur_nom;

	printf("Prenom ecrivain ->");
	fgets(Pteur_prenom, MAX_LENGTH, stdin);
	ecrivain->prenom = Pteur_prenom;

	printf("jour naissance ->");
	scanf_s("%d", &ecrivain->jour);

	printf("mois naissance ->");
	scanf_s("%d", &ecrivain->mois);

	printf("annee naissance ->");
	scanf_s("%d", &ecrivain->annee);
}

void insere_debut(ENTETE *Pteur_entete)
{
	NOEUD * Pteur_noeud;
	Pteur_noeud = creer_noeud();

	if (Pteur_entete->longueur == 0)
	{
		Pteur_entete->premier = Pteur_noeud;
		Pteur_entete->dernier = Pteur_noeud;
		Pteur_entete->longueur = 1;
	}
	else
	{
		Pteur_noeud->suivant = Pteur_entete->premier;
		Pteur_entete->premier = Pteur_noeud;
		Pteur_entete->longueur = Pteur_entete->longueur + 1;
	}
	return;
}

void affiche_list(ENTETE *Pteur_entete)
{
	int nbr_noeud, i;
	NOEUD * Pteur_noeud;

	nbr_noeud = Pteur_entete->longueur;
	if (nbr_noeud == 0)
	{
		printf("\n\t PAS DE NOEUD\n");
	}
	else
	{
		Pteur_noeud = Pteur_entete->premier;
		printf("\n");
		for (i = 1; i <= nbr_noeud; i++)
		{
			affiche_noeud(Pteur_noeud);
			Pteur_noeud = Pteur_noeud->suivant;
		}
		Sleep(2000);
	}
}

void affiche_noeud(NOEUD *Pteur_noeud)
{
	printf("**\t %s", Pteur_noeud->nom);
	printf("\t %s", Pteur_noeud->prenom);
	printf("\t %d/%d/%d\n\n", Pteur_noeud->jour, Pteur_noeud->mois, Pteur_noeud->annee);
}

void insere_fin(ENTETE * Pteur_entete)
{
	NOEUD * Pteur_noeud;
	Pteur_noeud = creer_noeud();
	if (Pteur_entete->longueur == 0)
	{
		Pteur_entete->premier = Pteur_noeud;
		Pteur_entete->dernier = Pteur_noeud;
		Pteur_entete->longueur = 1;
	}
	else
	{
		Pteur_entete->dernier->suivant = Pteur_noeud;
		Pteur_entete->longueur = Pteur_entete->longueur + 1;
	}
	return;
}

void supprime_elt(ENTETE * Pteur_entete)
{
	NOEUD * Pteur_RAZ, *Pteur_aval, *Pteur_amont;

	int choix, max, i;
	max = Pteur_entete->longueur;

	printf("choix de l'element a supprimer ->");
	scanf_s("%d", &choix);

	if (choix > max)
	{
		printf("\nMAUVAIS CHOIXXXX\n");
	}
	else
	{
		if (max == 1)
		{
			RAZ_noeud(Pteur_entete->premier);
			Pteur_entete->premier = NULL;
			Pteur_entete->dernier = NULL;
		}
		else if (choix == 1)
		{
			Pteur_RAZ = Pteur_entete->premier;
			Pteur_entete->premier = Pteur_RAZ->suivant;
			RAZ_noeud(Pteur_RAZ);
		}
		else if (choix == max)
		{
			Pteur_amont = Pteur_entete->premier;
			for (i = 1; i < max - 1; i++)
			{
				Pteur_amont = Pteur_amont->suivant;
			}
			Pteur_entete->dernier = Pteur_amont;
			Pteur_RAZ = Pteur_amont->suivant;
			Pteur_amont->suivant = NULL;
			RAZ_noeud(Pteur_RAZ);
		}
		else
		{
			Pteur_amont = Pteur_entete->premier;
			for (i = 1; i < max - 1; i++)
			{
				Pteur_amont = Pteur_amont->suivant;
			}
			Pteur_RAZ = Pteur_amont->suivant;
			Pteur_aval = Pteur_RAZ->suivant;
			Pteur_amont->suivant = Pteur_aval;
			RAZ_noeud(Pteur_RAZ);
		}
		Pteur_entete->longueur = Pteur_entete->longueur - 1;

	}
	return;


}

void RAZ_noeud(NOEUD *Pteur_nom)
{
	free(Pteur_nom->nom);
	free(Pteur_nom->prenom);
	free(Pteur_nom);
}

void RAZ_memoire(ENTETE * Pteur_entete)
{
	int nbr_elt, i;
	NOEUD * Pteur_noeud, *Pteur_RAZ;

	nbr_elt = Pteur_entete->longueur;

	Pteur_RAZ = Pteur_entete->premier;
	for (i = 1; i <= nbr_elt; i++)
	{
		Pteur_noeud = Pteur_RAZ->suivant;
		free(Pteur_RAZ);
		Pteur_RAZ = Pteur_noeud;
	}
	free(Pteur_entete);
	return;
}

void enregistrer_list(ENTETE * Pteur_entete)
{
	NOEUD * Pteur_noeud;

	errno_t erreur;
	FILE * mon_fichier = NULL;
	erreur = fopen_s(&mon_fichier, "list.txt", "w");

	
	if (erreur == 0)
	{
		Pteur_noeud = Pteur_entete->premier;
		while (Pteur_noeud != NULL)
		{
			memo_noeud(Pteur_noeud, mon_fichier);
			Pteur_noeud = Pteur_noeud->suivant;
		}
		fclose(mon_fichier);
	}
	else
	{
		printf("MARCHE PAS\n");
	}

}

void memo_noeud(NOEUD * Pteur_noeud,FILE * mon_fichier  )
{

	fputs(Pteur_noeud->nom, mon_fichier);
	fputs(Pteur_noeud->prenom, mon_fichier);
	fprintf(mon_fichier, "%d/%d/%d", Pteur_noeud->jour, Pteur_noeud->mois, Pteur_noeud->annee);
	fputc('\n', mon_fichier);
}

ENTETE * recup_list(void)
{
	NOEUD * Pteur_noeud =NULL;
	FILE * mon_fichier = NULL;
	errno_t erreur;
	ENTETE * Pteur_entete = NULL;

	erreur = fopen_s(&mon_fichier, "Recup.txt", "r");

	Pteur_entete = creer_entete();

	while (fin_fichier(mon_fichier) == false)
	{
		Pteur_noeud = charger_noeud(mon_fichier);
		charger_fin(Pteur_entete, Pteur_noeud);
	}

	return(Pteur_entete);

}

bool fin_fichier(FILE * mon_fichier)
{
	bool renvoi = false;
	if (fgetc(mon_fichier) == EOF)
	{
		renvoi = true;
	}
	else
	{
		fseek(mon_fichier, -1, SEEK_CUR);
	}
	return(renvoi);
}

void charger_fin(ENTETE *Pteur_entete, NOEUD*Pteur_noeud)
{
	NOEUD * Pteur_list;
	if (Pteur_entete->longueur == 0)
	{
		Pteur_entete->premier = Pteur_noeud;
		Pteur_entete->dernier = Pteur_noeud;
		Pteur_entete->longueur = 1;
	}
	else
	{
		Pteur_list = Pteur_entete->premier;
		while (Pteur_list->suivant != NULL)
		{
			Pteur_list = Pteur_list->suivant;
		}
		Pteur_list->suivant = Pteur_noeud;
		Pteur_entete->dernier= Pteur_noeud;
		Pteur_entete->longueur = Pteur_entete->longueur + 1;
	}
	return;
}

NOEUD * charger_noeud(FILE * mon_fichier)
{
	char * Pteur_nom, *Pteur_prenom;

	NOEUD * nouveau;

	if (nouveau = (NOEUD *)malloc(sizeof(NOEUD)))
	{

		Pteur_nom = (char *)malloc(MAX_LENGTH * sizeof(char));
		Pteur_prenom = (char *)malloc(MAX_LENGTH * sizeof(char));

		fgets(Pteur_nom, MAX_LENGTH, mon_fichier);
		nouveau->nom = Pteur_nom;

		fgets(Pteur_prenom, MAX_LENGTH, mon_fichier);
		nouveau->prenom = Pteur_prenom;
		
		fscanf_s(mon_fichier, "%d/%d/%d", &(nouveau->jour), &(nouveau->mois), &(nouveau->annee));
		nouveau->suivant = NULL;
	}
	return(nouveau);

}
